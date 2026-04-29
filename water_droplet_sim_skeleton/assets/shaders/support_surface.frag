#version 330 core
in vec3 vWorldNormal;
out vec4 FragColor;

uniform sampler2D uEnvironmentMap;

const float kPi = 3.14159265359;

vec2 equirectUv(vec3 dir) {
    vec3 d = normalize(dir);
    float u = atan(d.z, d.x) / (2.0 * kPi) + 0.5;
    float v = asin(clamp(d.y, -1.0, 1.0)) / kPi + 0.5;
    return vec2(u, v);
}

vec3 toneMap(vec3 color) {
    vec3 mapped = color / (color + vec3(1.0));
    return pow(mapped, vec3(1.0 / 2.2));
}

void main() {
    vec3 n = normalize(vWorldNormal);
    vec3 envAmbient = toneMap(texture(uEnvironmentMap, equirectUv(n)).rgb);
    vec3 base = vec3(0.34, 0.36, 0.34);
    vec3 ambient = mix(vec3(0.38), envAmbient, 0.62);
    FragColor = vec4(base * ambient, 1.0);
}
