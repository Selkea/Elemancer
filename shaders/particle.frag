#version 430 core

in vec3 vViewPos;

uniform vec3 uLightDir;   // view space
uniform vec3 uBaseColor;

out vec4 FragColor;

void main() {
    // Rebuild a sphere normal across the point sprite and drop the corners.
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(c, c);
    if (r2 > 1.0) discard;

    vec3 n = vec3(c.x, -c.y, sqrt(1.0 - r2));
    vec3 L = normalize(uLightDir);
    vec3 V = vec3(0.0, 0.0, 1.0);
    vec3 H = normalize(L + V);

    float diffuse = max(dot(n, L), 0.0);
    float specular = pow(max(dot(n, H), 0.0), 72.0);
    float fresnel = pow(1.0 - max(n.z, 0.0), 3.0);

    // Fade with distance so the far side of the body reads as further away.
    float depthFade = clamp((4.2 + vViewPos.z) / 2.4, 0.35, 1.0);

    vec3 color = uBaseColor * (0.18 + 0.82 * diffuse);
    color += vec3(1.0, 0.98, 0.94) * specular * 0.9;
    color += vec3(0.30, 0.55, 0.95) * fresnel * 0.5;

    FragColor = vec4(color * depthFade, 1.0);
}
