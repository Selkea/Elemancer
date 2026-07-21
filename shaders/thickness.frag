#version 430 core

// Additive pass: accumulate how much liquid each pixel looks through, which
// then drives Beer-Lambert absorption in the composite.
uniform float uRadius;

layout(location = 0) out float outThickness;

void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    c.y = -c.y;

    float r2 = dot(c, c);
    if (r2 > 1.0) discard;

    // Chord length straight through the sphere at this offset.
    outThickness = 2.0 * uRadius * sqrt(1.0 - r2);
}
