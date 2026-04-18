#version 330

// --------------------
// HLMS uniforms
// --------------------
uniform mat4 projmatrix; // actually proj*view
uniform mat4 viewmatrix;

uniform float fovy;
uniform vec4 vpsize;

// --------------------
// Vertex attributes
// --------------------
in vec3 position;      // location 0 (position)
in vec4 colour;      // location 1
in vec3 uv0;         // location 2
in vec3 uv1;         // location 3
in vec2 uv2;         // location 4
// --------------------
// Outputs
// --------------------
out vec2 texCoord;
out vec4 col;
out vec3 con;
out vec2 pixf;
// based on: https://github.com/kishimisu/Gaussian-Splatting-WebGL
// using precomputed terms as in https://github.com/cvlab-epfl/gaussian-splatting-web

vec3 computeCov2D(vec3 position, vec3 diag, vec3 upper, out float compensation)
{
    float tan_fovy = tan(fovy * 0.5);
    float tan_fovx = tan_fovy * vpsize.x / vpsize.y;

    float focal_y = vpsize.y / (2.0 * tan_fovy);
    float focal_x = vpsize.x / (2.0 * tan_fovx);

    // The following models the steps outlined by equations 29
	// and 31 in "EWA Splatting" (Zwicker et al., 2002).
	// Additionally considers aspect / scaling of viewport.

    vec4 t = viewmatrix * vec4(position, 1.0);

    float limx = 1.3 * tan_fovx;
    float limy = 1.3 * tan_fovy;
    float txtz = t.x / t.z;
    float tytz = t.y / t.z;
    t.x = clamp(txtz, -limx, limx) * t.z;
    t.y = clamp(tytz, -limy, limy) * t.z;

    mat3 J = mat3(
        focal_x / t.z, 0.0, -(focal_x * t.x) / (t.z * t.z),
        0.0, focal_y / t.z, -(focal_y * t.y) / (t.z * t.z),
        0.0, 0.0, 0.0
    );

    mat3 W = transpose(mat3(viewmatrix));

    mat3 T = W * J;

    mat3 Vrk = mat3(
        diag.x, upper.x, upper.y,
        upper.x, diag.y, upper.z,
        upper.y, upper.z, diag.z
    );

    mat3 cov = transpose(T) * Vrk * T;

    // Apply low-pass filter: every Gaussian should be at least
    // one pixel wide/high. Discard 3rd row and column.
    // aliasing correction
    // see: https://github.com/nerfstudio-project/gsplat/pull/117
    float detOrig = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];
    cov[0][0] += 0.3;
    cov[1][1] += 0.3;
    float detBlur = cov[0][0] * cov[1][1] - cov[0][1] * cov[0][1];
    compensation = sqrt(max(detOrig / detBlur, 0.0));

    return vec3(cov[0][0], cov[0][1], cov[1][1]);
}

void main()
{
	/*
   // simple shader for debugging.
    gl_Position = projmatrix * vec4(position, 1.0);
    gl_Position.xy += uv2 * 1; // 아주 작은 오프셋
    col = vec4(1, 0, 0, 1); // 빨간색 강제
	*/
    // Transform point by projecting
    vec4 p_hom = projmatrix * vec4(position, 1.0);
    vec3 p_proj = p_hom.xyz / p_hom.w;

    col = colour;

    float compensation;

    // Compute 2D screen-space covariance matrix
    vec3 cov = computeCov2D(position, uv0, uv1, compensation);

    col.a *= compensation;
    if (col.a < 1.0 / 255.0)
    {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // Invert covariance (EWA algorithm)
    float det = (cov.x * cov.z - cov.y * cov.y);
    float det_inv = 1. / det;
    vec3 conic = vec3(cov.z, -cov.y, cov.x) * det_inv;

    con = conic;

    // Compute extent in screen space (by finding eigenvalues of
    // 2D covariance matrix). Use extent to compute the bounding
    // rectangle of the splat in screen space.

    float mid = 0.5 * (cov.x + cov.z);
    float lambda1 = mid + sqrt(max(0.1, mid * mid - det));
    float lambda2 = mid - sqrt(max(0.1, mid * mid - det));
    float radius_px = ceil(3.0 * sqrt(max(lambda1, lambda2)));

	vec2 uv = vec2(uv2.x, -uv2.y);
	texCoord = uv * 0.5 + 0.5; // -1~1 → 0~1
    pixf = vec2(radius_px);
    float offsetSize = 2.0*radius_px; // diameter in pixels
	// quad (-1~1)
	vec2 pixelOffset = uv2 * offsetSize;
	//	pixel → clip space
	vec2 clipOffset = pixelOffset * (1.1 / vpsize.xy) * p_hom.w;
	//gl_Position = vec4(p_proj.xy+clipOffset, p_proj.z, 1.0);
    gl_Position = p_hom;
	gl_Position.xy += clipOffset;
}
