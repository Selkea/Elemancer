// Spliced into every fragment shader that needs the environment, just after
// the #version line. Both the background and the reflections read from this
// same function, so what the liquid mirrors always matches what is behind it.
//
// It is deliberately bright: water only reads as water when there is something
// with range for it to reflect and refract. Against a dark room it just looks
// like black goo.

uniform vec3 uLightDirWorld;

vec3 envColor(vec3 d) {
    d = normalize(d);
    vec3 sun = normalize(uLightDirWorld);

    vec3 zenith = vec3(0.20, 0.36, 0.62);
    vec3 horizon = vec3(0.66, 0.74, 0.84);
    vec3 ground = vec3(0.17, 0.16, 0.15);

    vec3 sky = mix(horizon, zenith, smoothstep(0.0, 0.65, d.y));
    vec3 color = mix(ground, sky, smoothstep(-0.12, 0.06, d.y));

    float cosSun = max(dot(d, sun), 0.0);
    color += vec3(1.0, 0.95, 0.85) * pow(cosSun, 320.0) * 6.0;  // disc
    color += vec3(1.0, 0.90, 0.75) * pow(cosSun, 12.0) * 0.22;  // haze

    return color;
}
