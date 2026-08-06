// --------------------
// Inputs (from vertex shader)
// --------------------
struct PS_INPUT
{
    float4 position : SV_POSITION;
    float2 sphereCoord : TEXCOORD0;
    float4 col         : COLOR0;
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
    if (dot(input.sphereCoord, input.sphereCoord) > 1.0f)
    {
        discard;
    }

    if (input.col.a < (1.0f / 255.0f))
    {
        discard;
    }

	//output.colour =float4(input.col.rgb, input.col.a);
	output.colour =float4(pow(input.col.rgb, 2.2f), input.col.a);
	//output.colour=float4(1.0,0.0,0.0,1.0);
	return output;
}
