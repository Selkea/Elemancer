#version 430 core

// Shade the smoothed depth buffer as a liquid surface: reconstruct position
// and normal from depth, then blend refraction against reflection by Fresnel
// and absorb light through the accumulated thickness.
in vec2 vUV;

uniform sampler2D uDepth;      // bilaterally smoothed view-space distance
uniform sampler2D uThickness;
uniform sampler2D uScene;      // environment already rendered behind the fluid

uniform mat4 uInvProj;
uniform mat4 uInvView;
uniform vec2 uTexel;
uniform vec3 uLightDirView;
uniform vec3 uLiquidColor;
uniform float uRefractScale;
uniform float uAbsorption;
uniform float uScatter;

layout(location = 0) out vec4 outColor;

const float kBackground = 1.0e6;

// de Greve 2006, "Reflections and Refractions in Ray Tracing": Snell's law in
// vector form, for eta = n1/n2. A negative radicand is total internal
// reflection, where no ray is transmitted at all.
vec3 refractDir(vec3 i, vec3 n, float eta) {
    float cosi = -dot(n, i);
    float k = 1.0 - eta * eta * (1.0 - cosi * cosi);
    if (k < 0.0) return vec3(0.0);
    return eta * i + (eta * cosi - sqrt(k)) * n;
}

vec3 viewPosFromDepth(vec2 uv, float dist) {
    vec4 clip = vec4(uv * 2.0 - 1.0, -1.0, 1.0);
    vec4 v = uInvProj * clip;
    vec3 dir = normalize(v.xyz / v.w);
    return dir * (dist / max(-dir.z, 1e-5));
}

void main() {
    float dist = texture(uDepth, vUV).r;
    vec3 background = texture(uScene, vUV).rgb;

    if (dist >= kBackground * 0.5) {
        outColor = vec4(background, 1.0);
        return;
    }

    vec3 P = viewPosFromDepth(vUV, dist);

    // One-sided differences, keeping whichever neighbour is nearer in depth.
    // A neighbour that landed on the background must be rejected outright:
    // differencing against 1e6 produces a garbage normal, which is what shows
    // up as black speckle along the silhouette. The derivative baseline is a
    // few texels wide, not one: a longer finite difference is far less
    // sensitive to per-pixel depth noise, which is what streaks the reflection
    // at the grazing rim where the surface is nearly tangent to view.
    vec2 sx = vec2(uTexel.x * 3.0, 0.0);
    vec2 sy = vec2(0.0, uTexel.y * 3.0);

    float dR = texture(uDepth, vUV + sx).r;
    float dL = texture(uDepth, vUV - sx).r;
    float dU = texture(uDepth, vUV + sy).r;
    float dD = texture(uDepth, vUV - sy).r;

    bool okR = dR < kBackground * 0.5;
    bool okL = dL < kBackground * 0.5;
    bool okU = dU < kBackground * 0.5;
    bool okD = dD < kBackground * 0.5;

    vec3 right = viewPosFromDepth(vUV + sx, dR) - P;
    vec3 left = P - viewPosFromDepth(vUV - sx, dL);
    vec3 ddx = vec3(sx.x, 0.0, 0.0);
    if (okR && okL) ddx = abs(left.z) < abs(right.z) ? left : right;
    else if (okR) ddx = right;
    else if (okL) ddx = left;

    vec3 up = viewPosFromDepth(vUV + sy, dU) - P;
    vec3 down = P - viewPosFromDepth(vUV - sy, dD);
    vec3 ddy = vec3(0.0, sy.y, 0.0);
    if (okU && okD) ddy = abs(down.z) < abs(up.z) ? down : up;
    else if (okU) ddy = up;
    else if (okD) ddy = down;

    vec3 N = normalize(cross(ddx, ddy));
    if (N.z < 0.0) N = -N;

    float thickness = texture(uThickness, vUV).r;
    vec3 V = normalize(-P);

    // Schlick, with R0 = ((n1-n2)/(n1+n2))^2 = 0.02 for an air/water interface.
    float fresnel = 0.02 + 0.98 * pow(1.0 - max(dot(N, V), 0.0), 5.0);

    // Fade the reflection out at the razor-thin rim. The bilateral blur cannot
    // smooth the outermost silhouette ring (its outer neighbour is background),
    // so the normal there stays noisy and, since Fresnel makes reflection
    // dominant at grazing, it streaks the sky. Suppressing reflection where the
    // film is thinnest lets the smooth refracted body show instead.
    fresnel *= smoothstep(0.0, 0.10, thickness);

    // Refraction: offset the background lookup along the actual refracted
    // ray rather than along the normal, so the bend responds to viewing angle
    // the way water does. Air into water, so eta = 1 / 1.333.
    vec3 T = refractDir(-V, N, 1.0 / 1.333);
    if (dot(T, T) < 1e-6) fresnel = 1.0;  // total internal reflection

    vec2 refractUV = clamp(vUV + T.xy * uRefractScale * min(thickness, 1.5),
                           vec2(0.001), vec2(0.999));
    vec3 refracted = texture(uScene, refractUV).rgb;
    refracted *= exp(-(vec3(1.0) - uLiquidColor) * thickness * uAbsorption);

    vec3 Pworld = (uInvView * vec4(P, 1.0)).xyz;
    vec3 Nworld = normalize((uInvView * vec4(N, 0.0)).xyz);
    vec3 Vworld = normalize((uInvView * vec4(V, 0.0)).xyz);

    // Glossy reflection: average the environment over a small cone rather than
    // taking a single mirror sample. At the grazing rim the reflected ray
    // sweeps fast across the bright sky, so a mirror sample turns any residual
    // normal wrinkle into a streak; spreading the taps smooths that out and
    // reads more like water than chrome.
    vec3 refl = reflect(-Vworld, Nworld);
    vec3 t1 = normalize(cross(Nworld, vec3(0.0, 1.0, 0.0)) + vec3(1e-4));
    vec3 t2 = cross(Nworld, t1);
    const float k = 0.06;
    vec3 reflected = (envColor(Pworld, refl) + envColor(Pworld, normalize(refl + k * t1)) +
                      envColor(Pworld, normalize(refl - k * t1)) +
                      envColor(Pworld, normalize(refl + k * t2)) +
                      envColor(Pworld, normalize(refl - k * t2))) *
                     0.2;

    vec3 color = mix(refracted, reflected, fresnel);

    // Light scattered back out of the body. Kept subtle: too much and the
    // liquid glows with its own colour instead of showing what is behind it.
    color += uLiquidColor * (1.0 - exp(-thickness * 1.2)) * uScatter;

    // Sun highlight. It is the single biggest source of the "streak" the eye
    // catches: it lands on the grazing top where the normal is noisiest, and a
    // sharp exponent shatters it into a striated band. Kept moderately broad,
    // and faded out at the thin rim where the normal is unreliable, so a clean
    // highlight only appears on the smooth interior.
    vec3 L = normalize(uLightDirView);
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 22.0) * smoothstep(0.0, 0.10, thickness);
    color += vec3(1.0, 0.97, 0.92) * spec * 0.5;

    // Fade the razor-thin rim into the background. The reconstructed normal is
    // unreliable at the silhouette (its outward neighbour is background), so it
    // streaks the reflection there; a real thin film of water is nearly
    // invisible anyway, so fading it both hides the artefact and softens the
    // edge.
    float edge = smoothstep(0.0, 0.05, thickness);
    color = mix(background, color, edge);

    outColor = vec4(color, 1.0);
}
