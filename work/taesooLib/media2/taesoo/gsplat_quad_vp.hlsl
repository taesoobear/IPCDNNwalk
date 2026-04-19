// --------------------
// HLMS uniforms
// --------------------

uniform float4x4 projmatrix;
uniform float4x4 viewmatrix;
uniform float4 vpsize;
uniform float    fovy;

// --------------------
// Vertex attributes
// --------------------
struct VS_INPUT
{
    float3 position : POSITION;    // location 0
    float4 colour   : COLOR0;      // location 1
    float3 uv0      : TEXCOORD0;   // location 2
    float3 uv1      : TEXCOORD1;   // location 3
    float2 uv2      : TEXCOORD2;   // location 4
};

// --------------------
// Outputs
// --------------------
struct VS_OUTPUT
{
    float4 position : SV_POSITION;
    float2 texCoord : TEXCOORD0;
    float4 col      : COLOR0;
    float3 con      : TEXCOORD1;
    float2 pixf     : TEXCOORD2;
};

// --------------------
// Helper: computeCov2D
// --------------------
float3 computeCov2D(float3 position, float3 diag, float3 upper, out float compensation)
{
    float tan_fovy = tan(fovy * 0.5);
    float tan_fovx = tan_fovy * vpsize.x / vpsize.y;

    float focal_y = vpsize.y / (2.0 * tan_fovy);
    float focal_x = vpsize.x / (2.0 * tan_fovx);

    // The following models the steps outlined by equations 29
    // and 31 in "EWA Splatting" (Zwicker et al., 2002).
    // Additionally considers aspect / scaling of viewport.

    float4 t = mul(viewmatrix, float4(position, 1.0));

    float limx = 1.3 * tan_fovx;
    float limy = 1.3 * tan_fovy;
    float txtz = t.x / t.z;
    float tytz = t.y / t.z;
    t.x = clamp(txtz, -limx, limx) * t.z;
    t.y = clamp(tytz, -limy, limy) * t.z;

    // HLSL은 column-major이므로 GLSL mat3(col0, col1, col2) 순서 주의
    // GLSL: mat3(c0r0, c0r1, c0r2,  c1r0, ...) → column-major
    // HLSL: float3x3 row-major 표기 → 아래는 transpose 적용 후 동일 결과
    float3x3 J = float3x3(
        focal_x / t.z,              0.0,  -(focal_x * t.x) / (t.z * t.z),
        0.0,              focal_y / t.z,  -(focal_y * t.y) / (t.z * t.z),
        0.0,                        0.0,   0.0
    );

    // GLSL: mat3(viewmatrix) → 상위 3x3, transpose()
    float3x3 viewUpper = float3x3(
        viewmatrix[0].xyz,
        viewmatrix[1].xyz,
        viewmatrix[2].xyz
    );
    float3x3 W = transpose(viewUpper);

    float3x3 T = mul(W, J);

    float3x3 Vrk = float3x3(
        diag.x,  upper.x, upper.y,
        upper.x, diag.y,  upper.z,
        upper.y, upper.z, diag.z
    );

    float3x3 cov = mul(transpose(T), mul(Vrk, T));

    // Apply low-pass filter / aliasing correction
    float detOrig = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];
    cov[0][0] += 0.3;
    cov[1][1] += 0.3;
    float detBlur = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];
    compensation = sqrt(max(detOrig / detBlur, 0.0));

    return float3(cov[0][0], cov[0][1], cov[1][1]);
}

// --------------------
// Vertex Shader Entry
// --------------------
VS_OUTPUT main(VS_INPUT input)
{
    VS_OUTPUT output;
	/*
	// simple shader for debugging.
	output.position    = mul(projmatrix, float4(input.position, 1.0));
    output.position.xy += input.uv2;

    output.texCoord = float2(0, 0);

	// 미사용 출력 초기화
    output.texCoord = float2(0.0, 0.0);
	output.col         = float4(1, 0, 0, 1); // 빨간색 강제
    output.con      = float3(0.0, 0.0, 0.0);
    output.pixf     = float2(0.0, 0.0);

    return output;
	*/
    // Transform point by projecting
    float4 p_hom  = mul(projmatrix, float4(input.position, 1.0));
    float3 p_proj = p_hom.xyz / p_hom.w;

    output.col = input.colour;

    float compensation;

    // Compute 2D screen-space covariance matrix
    float3 cov = computeCov2D(input.position, input.uv0, input.uv1, compensation);

    output.col.a *= compensation;
    if (output.col.a < 1.0 / 255.0)
    {
        output.position = float4(0.0, 0.0, 2.0, 1.0);
        // 나머지 출력은 더미값으로 채움 (HLSL은 early-return 후에도 구조체 반환 필요)
        output.texCoord = float2(0.0, 0.0);
        output.con      = float3(0.0, 0.0, 0.0);
        output.pixf     = float2(0.0, 0.0);
        return output;
    }

    // Invert covariance (EWA algorithm)
    float det     = (cov.x * cov.z - cov.y * cov.y);
    float det_inv = 1.0 / det;
    float3 conic  = float3(cov.z, -cov.y, cov.x) * det_inv;

    output.con = conic;

    // Compute extent in screen space (eigenvalues of 2D covariance matrix)
    float mid      = 0.5 * (cov.x + cov.z);
    float lambda1  = mid + sqrt(max(0.1, mid * mid - det));
    float lambda2  = mid - sqrt(max(0.1, mid * mid - det));
    float radius_px = ceil(3.0 * sqrt(max(lambda1, lambda2)));

    float2 uv       = float2(input.uv2.x, -input.uv2.y);
    output.texCoord = uv * 0.5 + 0.5;  // -1~1 → 0~1
    output.pixf     = float2(radius_px, radius_px);

    float offsetSize  = 2.0 * radius_px;           // diameter in pixels
    float2 pixelOffset = input.uv2 * offsetSize;
    float2 clipOffset  = pixelOffset * (1.1 / vpsize.xy) * p_hom.w;

    output.position    = p_hom;
    output.position.xy += clipOffset;

    return output;
}
