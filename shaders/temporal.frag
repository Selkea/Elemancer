#version 430 core

// Temporal smoothing of the reconstructed surface depth. The screen-space
// surface is rebuilt from particle positions every frame, so when the body
// moves -- especially when it rotates -- the per-particle lumpiness slides
// around and the surface boils. Blending each pixel toward the previous
// frame's depth stabilises it.
//
// The disocclusion guard is what keeps this from smearing: where the surface
// actually moved a lot between frames (a fast flick, or the silhouette
// sweeping past), the history is rejected and the current frame is trusted, so
// only genuine frame-to-frame jitter is damped.
in vec2 vUV;

uniform sampler2D uCurrent;   // this frame's spatially smoothed depth
uniform sampler2D uHistory;   // last frame's resolved depth
uniform float uBlend;         // fraction of history kept, 0 disables
uniform float uMaxDelta;      // reject history beyond this depth difference

layout(location = 0) out float outDepth;

const float kBackground = 1.0e6;

void main() {
    float c = texture(uCurrent, vUV).r;
    if (c >= kBackground * 0.5) {
        outDepth = c;
        return;
    }

    float h = texture(uHistory, vUV).r;
    if (h >= kBackground * 0.5 || abs(h - c) > uMaxDelta) {
        outDepth = c;   // disoccluded or moved too far: trust the new frame
        return;
    }

    outDepth = mix(c, h, uBlend);
}
