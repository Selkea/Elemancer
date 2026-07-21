#version 430 core

in vec2 vUV;

uniform mat4 uInvProj;
uniform mat4 uInvView;

layout(location = 0) out vec4 outColor;

void main() {
    vec4 clip = vec4(vUV * 2.0 - 1.0, -1.0, 1.0);
    vec4 v = uInvProj * clip;
    vec3 dirView = normalize(v.xyz / v.w);
    vec3 dirWorld = normalize((uInvView * vec4(dirView, 0.0)).xyz);

    outColor = vec4(envColor(dirWorld), 1.0);
}
