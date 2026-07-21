#version 430 core

// Carve a real sphere out of each point sprite and record its view-space
// distance. Writing gl_FragDepth as well keeps overlapping spheres sorting
// against each other correctly.
in vec3 vViewPos;

uniform mat4 uProj;
uniform float uRadius;

layout(location = 0) out float outDepth;

void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    c.y = -c.y;

    float r2 = dot(c, c);
    if (r2 > 1.0) discard;

    vec3 normal = vec3(c, sqrt(1.0 - r2));
    vec3 fragViewPos = vViewPos + normal * uRadius;

    vec4 clip = uProj * vec4(fragViewPos, 1.0);
    gl_FragDepth = (clip.z / clip.w) * 0.5 + 0.5;

    outDepth = -fragViewPos.z;
}
