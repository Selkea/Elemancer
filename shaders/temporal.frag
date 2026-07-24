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
uniform vec2 uHistoryShift;   // the body's screen-space motion since last frame

layout(location = 0) out float outDepth;

const float kBackground = 1.0e6;

void main() {
    float c = texture(uCurrent, vUV).r;
    if (c >= kBackground * 0.5) {
        outDepth = c;
        return;
    }

    // Reproject: a surface point at vUV this frame was at vUV - shift last frame,
    // shift being how far the body moved on screen. Without this, blending the
    // history sampled straight at vUV mixes the surface at its old position with
    // the new one; for a translating body that paints trailing concentric arcs
    // (the depth iso-contours of the two offset surfaces) and drags the rim
    // lumpiness into ghosts. Aligning the history to the moved body removes that
    // while keeping the still-body smoothing. Estimated from the centroid, so it
    // tracks translation; rotation and deformation are left to the guard below.
    vec2 histUV = vUV - uHistoryShift;
    float h = texture(uHistory, histUV).r;
    bool offscreen = any(lessThan(histUV, vec2(0.0))) || any(greaterThan(histUV, vec2(1.0)));
    if (offscreen || h >= kBackground * 0.5 || abs(h - c) > uMaxDelta) {
        outDepth = c;   // disoccluded or moved too far: trust the new frame
        return;
    }

    outDepth = mix(c, h, uBlend);
}
