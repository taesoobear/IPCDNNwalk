#version 330

in vec2 sphereCoord;
in vec4 col;

out vec4 outColour;

void main()
{
// outColour = vec4(1.0, 0.0, 0.0, 1.0);
    if (dot(sphereCoord, sphereCoord) > 1.0)
    {
        discard;
    }

    if (col.a < 1.0 / 255.0)
    {
        discard;
    }

    outColour = col;
}
