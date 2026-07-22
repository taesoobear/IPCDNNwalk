//#include <metal_stdlib>
using namespace metal;

// --------------------
// Inputs (from vertex shader)
// --------------------
struct PS_INPUT
{
    float2 sphereCoord [[user(locn0)]];
    float4 col      [[user(locn1)]];
};

// --------------------
// Output
// --------------------
fragment float4 main_metal(PS_INPUT in [[stage_in]])
{
	if (dot(in.sphereCoord, in.sphereCoord) > 1.0f)
	{
		discard_fragment();
	}

	if (in.col.a < (1.0f / 255.0f))
	{
		discard_fragment();
	}


    return float4(pow(in.col.rgb, float3(2.2)), in.col.a);
}
