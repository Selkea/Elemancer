#version 430 core

// Separable bilateral blur over view-space depth. The depth-difference weight
// is what stops the filter smearing across silhouettes: without it, separate
// blobs would melt into each other wherever they overlap on screen.
in vec2 vUV;

uniform sampler2D uDepth;
uniform vec2 uDir;           // one texel step along the blur axis
uniform float uRadius;       // taps either side
uniform float uSigmaSpatial;
uniform float uSigmaDepth;

layout(location = 0) out float outDepth;

const float kBackground = 1.0e6;

void main() {
    float d = texture(uDepth, vUV).r;
    if (d >= kBackground * 0.5) {
        outDepth = d;
        return;
    }

    float sum = 0.0;
    float wsum = 0.0;
    int taps = int(uRadius);

    for (int i = -taps; i <= taps; ++i) {
        float s = texture(uDepth, vUV + uDir * float(i)).r;
        if (s >= kBackground * 0.5) continue;

        float fi = float(i);
        float ws = exp(-(fi * fi) / (2.0 * uSigmaSpatial * uSigmaSpatial));
        float dz = s - d;
        float wd = exp(-(dz * dz) / (2.0 * uSigmaDepth * uSigmaDepth));

        float w = ws * wd;
        sum += s * w;
        wsum += w;
    }

    outDepth = wsum > 0.0 ? sum / wsum : d;
}
