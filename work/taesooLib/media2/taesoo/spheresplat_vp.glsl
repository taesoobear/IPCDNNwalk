#version 330

// --------------------
// HLMS uniforms
// --------------------
uniform mat4 projmatrix; // actually proj*view
uniform mat4 viewmatrix;

uniform float fovy;
uniform vec4 vpsize;     // xy = viewport width, height

// --------------------
// Vertex attributes
// --------------------
in vec3 position;      // location 0 (position)
in vec4 colour;      // rgba
in vec2 uv0;
in vec2 uv1;         // quad corner: (-1,-1) ~ (1,1),

// --------------------
// Outputs
// --------------------
out vec2 sphereCoord;
out vec4 col;

void main()
{    
	/*
   // for easier debugging

	col = vec4(1.0, 0.0, 0.0, 1.0);
    texCoord = uv1;

    vec4 centerClip = projmatrix * vec4(position, 1.0);

    gl_Position = centerClip;

    // 화면에서 확실히 보이도록 큰 고정 크기
    gl_Position.xy += uv1 * 0.05 * centerClip.w;
    //gl_Position.xy += uv1 ;
	/*
    col = colour;

	*/

	col = colour;
	float radius=uv0.x;

    // Sphere center in view space
    vec4 centerViewHom = viewmatrix * vec4(position, 1.0);
    vec3 centerView = centerViewHom.xyz;


    // Sphere center in clip space
    vec4 centerClip = projmatrix * vec4(position, 1.0);

    // 카메라 뒤 또는 near plane에 지나치게 가까운 sphere 제거
    if (centerClip.w <= 0.0 || radius <= 0.0 || col.a < 1.0 / 255.0)
    {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        sphereCoord = vec2(0.0);
        return;
    }

    //
    // Perspective projection:
    //
    // focal_y = viewportHeight / (2 tan(fovy / 2))
    // radiusPixels = focal_y * worldRadius / viewDepth
    //
    // 일반적인 OpenGL view space에서는 카메라 앞쪽의 z가 음수이므로
    // depth = -centerView.z를 사용합니다.
    //
    float focalY = vpsize.y / (2.0 * tan(fovy * 0.5));
    float viewDepth = max(-centerView.z, 1e-6);

    float radiusPx = focalY * radius / viewDepth;

    // 너무 작은 sphere도 최소 약 1 pixel로 표시하려면 사용
    radiusPx = max(radiusPx, 0.5);

    // uv1는 (-1,-1) ~ (1,1)
    sphereCoord = vec2(uv1.x, -uv1.y);
    //texCoord = sphereCoord * 0.5 + 0.5;

    //
    // uv1가 이미 -1~1이므로 radiusPx만 곱해야 합니다.
    // 기존 코드의 2 * radius_px는 quad가 예상보다 두 배 커질 수 있습니다.
    //
    vec2 pixelOffset = uv1 * radiusPx;

    // pixel offset -> NDC offset
    vec2 ndcOffset = pixelOffset * (2.0 / vpsize.xy);

    // NDC offset -> clip-space offset
    vec2 clipOffset = ndcOffset * centerClip.w;

    gl_Position = centerClip;
    gl_Position.xy += clipOffset;
}
