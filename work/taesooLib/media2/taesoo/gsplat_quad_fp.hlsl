// --------------------
// Inputs (from vertex shader)
// --------------------
struct PS_INPUT
{
    float4 position : SV_POSITION;
    float2 texCoord : TEXCOORD0;
    float4 col      : COLOR0;
    float3 con      : TEXCOORD1;
    float2 pixf     : TEXCOORD2;
};

// --------------------
// Output
// --------------------
struct PS_OUTPUT
{
    float4 colour : SV_TARGET;
};

PS_OUTPUT main(PS_INPUT input)
{
    PS_OUTPUT output;

    //output.colour= float4(1.0, 0.0, 0.0, 1.0); // 빨간색
	//return output;


    float2 d = (input.texCoord * 2.0 - 1.0) * -input.pixf;
    d.x *= -1.0;

    // Resample using conic matrix (cf. "Surface
    // Splatting" by Zwicker et al., 2001)
    float power = -0.5 * (input.con.x * d.x * d.x + input.con.z * d.y * d.y) - input.con.y * d.x * d.y;

    if (power > 0.0)
    {
        discard;
    }

    // Eq. (2) from 3D Gaussian splatting paper.
    float alpha = min(0.99, input.col.a * exp(power));

    // we render in reverse order, so we can use normal alpha blending
    output.colour = float4(pow(input.col.rgb, float3(2.2, 2.2, 2.2)), alpha); // pow: linear -> gamma

    return output;
}
