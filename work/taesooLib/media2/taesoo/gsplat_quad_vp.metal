//#include <metal_stdlib>
using namespace metal;

// --------------------
// Uniforms
// --------------------
struct Uniforms
{
    float4x4 projmatrix;
    float4x4 viewmatrix;
    float    fovy;
    float4   vpsize;
};

// --------------------
// Vertex attributes
// --------------------
//const uint32 MetalVaoManager::VERTEX_ATTRIBUTE_INDEX[VES_COUNT] = {
//        0,  // VES_POSITION - 1
//        3,  // VES_BLEND_WEIGHTS - 1
//        4,  // VES_BLEND_INDICES - 1
//        1,  // VES_NORMAL - 1
//        5,  // VES_DIFFUSE - 1
//        6,  // VES_SPECULAR - 1
struct VS_INPUT
{
    float3 position [[attribute(0)]];
    float4 colour   [[attribute(5)]]; // VES_DIFFUSE
    float3 uv0      [[attribute(7)]];
    float3 uv1      [[attribute(8)]];
    float2 uv2      [[attribute(9)]];
};

// --------------------
// Outputs → Fragment
// --------------------
struct VS_OUTPUT
{
    float4 position [[position]];
    float2 texCoord [[user(locn0)]];
    float4 col      [[user(locn1)]];
    float3 con      [[user(locn2)]];
    float2 pixf     [[user(locn3)]];
};

// --------------------
// computeCov2D
// --------------------
float3 computeCov2D(float3 position,
                    float3 diag,
                    float3 upper,
                    constant Uniforms& u,
                    thread float &compensation)
{
    float tan_fovy = tan(u.fovy * 0.5);
    float tan_fovx = tan_fovy * u.vpsize.x / u.vpsize.y;

    float focal_y = u.vpsize.y / (2.0 * tan_fovy);
    float focal_x = u.vpsize.x / (2.0 * tan_fovx);

    float4 t = u.viewmatrix * float4(position, 1.0);

    float limx = 1.3 * tan_fovx;
    float limy = 1.3 * tan_fovy;

    float txtz = t.x / t.z;
    float tytz = t.y / t.z;

    t.x = clamp(txtz, -limx, limx) * t.z;
    t.y = clamp(tytz, -limy, limy) * t.z;

    float3x3 J = float3x3(
        float3(focal_x / t.z, 0.0, -(focal_x * t.x) / (t.z * t.z)),
        float3(0.0, focal_y / t.z, -(focal_y * t.y) / (t.z * t.z)),
        float3(0.0, 0.0, 0.0)
    );
	float3x3 W = transpose(float3x3(
		u.viewmatrix[0].xyz,
		u.viewmatrix[1].xyz,
		u.viewmatrix[2].xyz
	));

    float3x3 T = W * J;

    float3x3 Vrk = float3x3(
        float3(diag.x, upper.x, upper.y),
        float3(upper.x, diag.y, upper.z),
        float3(upper.y, upper.z, diag.z)
    );

    float3x3 cov = transpose(T) * Vrk * T;

    float detOrig = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];

    cov[0][0] += 0.3;
    cov[1][1] += 0.3;

    float detBlur = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];

    compensation = sqrt(max(detOrig / detBlur, 0.0));

    return float3(cov[0][0], cov[0][1], cov[1][1]);
}

// --------------------
// Vertex Main
// --------------------
vertex VS_OUTPUT main_metal(
//		uint vertexID [[vertex_id]],           
//		constant VS_INPUT*       vb       [[buffer(0)]],      
	VS_INPUT in [[stage_in]],
		constant Uniforms& u [[buffer(PARAMETER_SLOT)]])
{
//	VS_INPUT in=vb[vertexID];
    VS_OUTPUT out;

/*
	// for debugging
	float4 pos = u.projmatrix * float4(in.position, 1.0);
    // offset (same as GLSL)
    pos.xy += in.uv2 * 1;

    out.position = pos;

    // force red color
    //out.col = float4(1.0, 0.0, 0.0, 1.0);
    //out.col = float4(in.colour)/255.0;
    out.col = in.colour;
	return out;
	*/
    float4 p_hom = u.projmatrix * float4(in.position, 1.0);
    float3 p_proj = p_hom.xyz / p_hom.w;

    out.col = in.colour;

    float compensation;
    float3 cov = computeCov2D(in.position, in.uv0, in.uv1, u, compensation);

    out.col.a *= compensation;

    if (out.col.a < (1.0 / 255.0))
    {
        out.position = float4(0.0, 0.0, 2.0, 1.0);
        return out;
    }

    float det = (cov.x * cov.z - cov.y * cov.y);
    float det_inv = 1.0 / det;

    float3 conic = float3(cov.z, -cov.y, cov.x) * det_inv;
    out.con = conic;

    float mid = 0.5 * (cov.x + cov.z);
    float lambda1 = mid + sqrt(max(0.1, mid * mid - det));
    float lambda2 = mid - sqrt(max(0.1, mid * mid - det));

    float radius_px = ceil(3.0 * sqrt(max(lambda1, lambda2)));

    float2 uv = float2(in.uv2.x, -in.uv2.y);
    out.texCoord = uv * 0.5 + 0.5;

    out.pixf = float2(radius_px);

    float offsetSize = 2.0 * radius_px;
    float2 pixelOffset = in.uv2 * offsetSize;

    float2 clipOffset = pixelOffset * (1.1 / u.vpsize.xy) * p_hom.w;

    out.position = p_hom;
    out.position.xy += clipOffset;

    return out;
}
