#version 430 core

// Diffuse spray droplets. xyz is position, w is remaining life.
layout(location = 0) in vec4 aPosLife;

uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointScale;
uniform float uRadius;
uniform float uLifeMax;

out float vFade;

void main() {
    vec4 viewPos = uView * vec4(aPosLife.xyz, 1.0);
    gl_Position = uProj * viewPos;

    // Never let a droplet fall below a pixel, or the spray flickers as
    // individual specks wink in and out between frames.
    gl_PointSize = max(uPointScale * uRadius / max(-viewPos.z, 0.001), 1.5);

    vFade = clamp(aPosLife.w / uLifeMax, 0.0, 1.0);
}
