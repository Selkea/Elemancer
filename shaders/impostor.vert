#version 430 core

// Shared by the depth and thickness passes: one point sprite per particle,
// sized so it covers exactly the sphere's projected diameter.
layout(location = 0) in vec3 aPos;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointScale;  // framebufferHeight * proj[1][1]
uniform float uRadius;

out vec3 vViewPos;

void main() {
    vec4 viewPos = uView * vec4(aPos, 1.0);
    vViewPos = viewPos.xyz;

    gl_Position = uProj * viewPos;
    gl_PointSize = uPointScale * uRadius / max(-viewPos.z, 0.001);
}
