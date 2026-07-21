#version 430 core

in float vFade;

uniform vec3 uColor;
uniform float uIntensity;

layout(location = 0) out vec4 outColor;

void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(c, c);
    if (r2 > 1.0) discard;

    // Soft round droplet, brightest in the middle, fading out over the tail
    // of its life so it never pops.
    float falloff = (1.0 - r2) * (1.0 - r2);
    float a = falloff * smoothstep(0.0, 0.35, vFade) * uIntensity;

    // Premultiplied, drawn with additive blending.
    outColor = vec4(uColor * a, a);
}
