#version 330

// --------------------
// Inputs (from vertex shader)
// --------------------
in vec2 texCoord;
in vec4 col;
in vec3 con;
in vec2 pixf;
// --------------------
// Output
// --------------------
out vec4 outColour;

void main()
{
    vec2 d = (texCoord*2.0 - 1.0) * -pixf;
	d.x *= -1.0;
    // Resample using conic matrix (cf. "Surface
    // Splatting" by Zwicker et al., 2001)
    float power = -0.5 * (con.x * d.x * d.x + con.z * d.y * d.y) - con.y * d.x * d.y;

    if (power > 0.0)
    {
        discard;
    }

    // Eq. (2) from 3D Gaussian splatting paper.
    float alpha = min(.99, col.a * exp(power));
	//alpha=1.0;

    // we render in reverse order, so we can use normal alpha blending
    outColour = vec4(pow(col.rgb, vec3(2.2)), alpha); // pow: linear -> gamma
	//outColour = vec4(con * 0.5 + 0.5, 1.0);
	//outColour = vec4(pixf / 100.0, 0.0, 1.0);
	//outColour = vec4(texCoord, 0.0, 1.0);

}
