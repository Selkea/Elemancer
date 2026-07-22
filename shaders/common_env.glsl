// Shared environment. Both the background pass and the liquid's reflections
// read from this same code, so what the water mirrors always matches what is
// actually behind it.
//
// It is ray-origin aware -- envColor(ro, rd) -- so the finite floor plane is
// intersected correctly whether the ray starts at the camera (background) or
// at a point on the water surface (reflection). A varied environment is what
// makes clear water read as clear: it needs detail to lens and mirror.

uniform vec3 uLightDirWorld;
uniform float uTime;

const float kFloorY = -1.9;  // must match FluidParams::floorY so the liquid pools on it

float hash21(vec2 p) {
    p = fract(p * vec2(123.34, 345.45));
    p += dot(p, p + 34.345);
    return fract(p.x * p.y);
}

float vnoise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    float a = hash21(i);
    float b = hash21(i + vec2(1.0, 0.0));
    float c = hash21(i + vec2(0.0, 1.0));
    float d = hash21(i + vec2(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

float fbm(vec2 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 5; ++i) {
        v += a * vnoise(p);
        p = p * 2.02 + 11.0;
        a *= 0.5;
    }
    return v;
}

vec3 skyColor(vec3 rd, vec3 sun) {
    float t = clamp(rd.y, 0.0, 1.0);
    vec3 zenith = vec3(0.16, 0.33, 0.62);
    vec3 horizon = vec3(0.74, 0.82, 0.92);
    vec3 col = mix(horizon, zenith, pow(t, 0.55));

    // Clouds, projected onto a plane overhead and drifting slowly. Faded out
    // near the horizon, where the projection would otherwise shear and shimmer.
    if (rd.y > 0.03) {
        vec2 cp = rd.xz / rd.y;
        cp = cp * 0.6 + vec2(0.02, 0.01) * uTime;
        float n = fbm(cp);
        float cover = smoothstep(0.50, 0.95, n) * smoothstep(0.03, 0.30, rd.y);
        vec3 cloudCol = mix(vec3(0.52, 0.55, 0.63), vec3(1.02, 1.00, 0.98), n);
        col = mix(col, cloudCol, cover * 0.9);
    }

    float s = max(dot(rd, sun), 0.0);
    col += vec3(1.0, 0.95, 0.85) * pow(s, 340.0) * 8.0;   // disc
    col += vec3(1.0, 0.90, 0.75) * pow(s, 12.0) * 0.20;   // haze
    return col;
}

vec3 floorColor(vec3 ro, vec3 rd, vec3 sun) {
    float t = (kFloorY - ro.y) / rd.y;
    vec3 p = ro + rd * t;
    vec2 g = p.xz;

    // Checker with a subtle brighter grid line on the seams.
    float chk = mod(floor(g.x) + floor(g.y), 2.0);
    vec3 base = mix(vec3(0.10, 0.11, 0.13), vec3(0.19, 0.21, 0.25), chk);
    vec2 seam = abs(fract(g) - 0.5);
    base += smoothstep(0.46, 0.5, max(seam.x, seam.y)) * 0.06;

    // A dim mirror of the sky, so the floor looks faintly wet.
    vec3 skyRefl = skyColor(reflect(rd, vec3(0.0, 1.0, 0.0)), sun);
    base = mix(base, skyRefl, 0.12);

    // Fade to the horizon colour with distance so the pattern does not alias
    // away into the far field.
    float dist = length(p - ro);
    float fog = 1.0 - exp(-dist * 0.035);
    return mix(base, vec3(0.74, 0.82, 0.92), fog);
}

vec3 envColor(vec3 ro, vec3 rd) {
    rd = normalize(rd);
    vec3 sun = normalize(uLightDirWorld);
    if (rd.y < -0.002 && ro.y > kFloorY) return floorColor(ro, rd, sun);
    return skyColor(rd, sun);
}

// Direction-only overload for callers that treat the environment as infinitely
// distant (an origin at the world centre is a fine approximation for those).
vec3 envColor(vec3 rd) {
    return envColor(vec3(0.0), rd);
}
