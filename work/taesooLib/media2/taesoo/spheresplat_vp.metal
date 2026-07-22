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
    float3 position [[attribute(VES_POSITION)]];
    float4 colour   [[attribute(VES_DIFFUSE)]]; 
    float2 radius   [[attribute(VES_TEXTURE_COORDINATES0)]];
    float2 uv0      [[attribute(VES_TEXTURE_COORDINATES1)]];
};

// --------------------
// Outputs → Fragment
// --------------------
struct VS_OUTPUT
{
    float4 position [[position]];
    float2 sphereCoord [[user(locn0)]];
	float4 col [[user(locn1)]];
};

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
    pos.xy += in.uv0 * 1;

    out.position = pos;

    // force red color
    //out.col = float4(1.0, 0.0, 0.0, 1.0);
    //out.col = float4(in.colour)/255.0;
    out.col = in.colour;
	return out;
	*/
    out.col = in.colour;

    const float radius = in.radius.x;

    // Sphere center in view space
    const float4 centerViewHom =
        u.viewmatrix * float4(in.position, 1.0);

    const float3 centerView = centerViewHom.xyz;

    // Sphere center in clip space
    const float4 centerClip =
        u.projmatrix * float4(in.position, 1.0);

    // 카메라 뒤, 반지름 0 이하 또는 완전히 투명한 sphere 제거
    if (centerClip.w <= 0.0f ||
        radius <= 0.0f )
		//|| out.col.a < (1.0f / 255.0f))
    {
        // 화면 밖으로 이동
        out.position = float4(0.0f, 0.0f, 2.0f, 1.0f);
        out.sphereCoord = float2(0.0f);

        return out;
    }

    // Perspective projection

    // focalY =
    //     viewportHeight / (2 * tan(fovy / 2))

    // radiusPixels =
    //     focalY * worldRadius / viewDepth

    // OpenGL/Ogre와 동일하게 카메라 앞쪽의 view-space z가
    // 음수라고 가정합니다.
    const float focalY =
        u.vpsize.y /
        (2.0f * tan(u.fovy * 0.5f));

    const float viewDepth =
        max(-centerView.z, 1.0e-6f);

    float radiusPx =
        focalY * radius / viewDepth;

    // 너무 작은 sphere도 최소 반 픽셀 크기로 표시
    radiusPx = max(radiusPx, 0.5f);

    // Quad local coordinate
    out.sphereCoord =
        float2(in.uv0.x, -in.uv0.y);

    // uv0은 이미 -1 ~ 1
    const float2 pixelOffset =
        in.uv0 * radiusPx;

    // Pixel offset -> NDC offset
    const float2 ndcOffset =
        pixelOffset *
        (2.0f / u.vpsize.xy);

    // NDC offset -> clip-space offset
    const float2 clipOffset =
        ndcOffset * centerClip.w;

    out.position = centerClip;
    out.position.xy += clipOffset;

    return out;
}
