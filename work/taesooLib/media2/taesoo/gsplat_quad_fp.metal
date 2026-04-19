//#include <metal_stdlib>
using namespace metal;

// --------------------
// Inputs (from vertex shader)
// --------------------
struct PS_INPUT
{
    float2 texCoord [[user(locn0)]];
    float4 col      [[user(locn1)]];
    float3 con      [[user(locn2)]];
    float2 pixf     [[user(locn3)]];
};

// --------------------
// Output
// --------------------
fragment float4 main_metal(PS_INPUT in [[stage_in]])
{
    float2 d = (in.texCoord * 2.0 - 1.0) * -in.pixf;
    d.x *= -1.0;

    // Gaussian power
    float power = -0.5 * (in.con.x * d.x * d.x +
                          in.con.z * d.y * d.y)
                  - in.con.y * d.x * d.y;

    if (power > 0.0)
    {
        discard_fragment();
    }

    float alpha = min(0.99, in.col.a * exp(power));

    return float4(pow(in.col.rgb, float3(2.2)), alpha);
}
