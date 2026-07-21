#version 430 core

layout(location = 0) in vec3 aPos;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointScale;  // framebufferHeight * proj[1][1]
uniform float uRadius;      // particle radius, world units

out vec3 vViewPos;

void main() {
    vec4 viewPos = uView * vec4(aPos, 1.0);
    vViewPos = viewPos.xyz;

    gl_Position = uProj * viewPos;
    gl_PointSize = uPointScale * uRadius / max(-viewPos.z, 0.001);
}
