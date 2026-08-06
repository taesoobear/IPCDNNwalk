// --------------------
// HLMS uniforms
// --------------------

uniform float4x4 projmatrix;
uniform float4x4 viewmatrix;
uniform float    fovy;
uniform float4 vpsize;

// --------------------
// Vertex attributes
// --------------------
struct VS_INPUT
{
    float3 position : POSITION;
    float4 colour   : COLOR0;
    float2 radius   : TEXCOORD0;
    float2 uv0      : TEXCOORD1;
};

// --------------------
// Outputs
// --------------------
struct VS_OUTPUT
{
    float4 position    : SV_Position;
    float2 sphereCoord : TEXCOORD0;
    float4 col         : COLOR0;
};

// --------------------
// Vertex Main
// --------------------
VS_OUTPUT main(VS_INPUT input)
{
    VS_OUTPUT output;
	/*
	// simple shader for debugging.
	output.position    = mul(projmatrix, float4(input.position, 1.0));
    output.position.xy += input.uv0;

    output.sphereCoord = float2(0, 0);

	// 미사용 출력 초기화
    output.sphereCoord = float2(0.0, 0.0);
	output.col         = float4(1, 0,0, 1); // 빨간색 강제

    return output;
	*/

    output.col = input.colour;

    const float radius = input.radius.x;

    // Sphere center in view space
    const float4 centerViewHom =
        mul(viewmatrix, float4(input.position, 1.0f));

    const float3 centerView =
        centerViewHom.xyz;

    // Sphere center in clip space
    const float4 centerClip =
        mul(projmatrix, float4(input.position, 1.0f));

    // 카메라 뒤에 있거나 반지름이 0 이하인 sphere 제거
    if (centerClip.w <= 0.0f ||
        radius <= 0.0f)
    {
        // 화면 밖으로 이동
        output.position =
            float4(0.0f, 0.0f, 2.0f, 1.0f);

        output.sphereCoord =
            float2(0.0f, 0.0f);

        return output;
    }

    // Perspective projection
    //
    // focalY =
    //     viewportHeight / (2 * tan(fovy / 2))
    //
    // radiusPixels =
    //     focalY * worldRadius / viewDepth
    //
    // 카메라 앞쪽의 view-space z가 음수라고 가정
    const float focalY =
        vpsize.y /
        (2.0f * tan(fovy * 0.5f));

    const float viewDepth =
        max(-centerView.z, 1.0e-6f);

    float radiusPx =
        focalY * radius / viewDepth;

    // 너무 작은 sphere도 최소 반 픽셀 크기로 표시
    radiusPx =
        max(radiusPx, 0.5f);

    // Quad local coordinate
    output.sphereCoord =
        float2(input.uv0.x, -input.uv0.y);

    // uv0 범위는 -1 ~ 1
    const float2 pixelOffset =
        input.uv0 * radiusPx;

    // Pixel offset -> NDC offset
    const float2 ndcOffset =
        pixelOffset *
        (2.0f / vpsize.xy);

    // NDC offset -> clip-space offset
    const float2 clipOffset =
        ndcOffset * centerClip.w;

    output.position = centerClip;
    output.position.xy += clipOffset;

    return output;
}
