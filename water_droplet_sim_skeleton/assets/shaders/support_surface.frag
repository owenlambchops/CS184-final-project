#version 330 core
in vec3 vWorldPosition;
in vec3 vWorldNormal;
out vec4 FragColor;

uniform sampler2D uEnvironmentMap;
uniform vec3 uCameraPos;
uniform float uIor;
uniform float uOpacity;
uniform vec3 uTintColor;

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
    vec3 viewDir = normalize(vWorldPosition - uCameraPos);
    vec3 envAmbient = toneMap(texture(uEnvironmentMap, equirectUv(n)).rgb);
    vec3 reflected = toneMap(texture(uEnvironmentMap, equirectUv(reflect(viewDir, n))).rgb);

    float ndotv = clamp(dot(n, -viewDir), 0.0, 1.0);
    float eta = max(uIor, 1.0001);
    float f0 = pow((1.0 - eta) / (1.0 + eta), 2.0);
    float fresnel = f0 + (1.0 - f0) * pow(1.0 - ndotv, 5.0);

    vec3 tintedTransmission = uTintColor * mix(vec3(0.55), envAmbient, 0.45);
    vec3 color = mix(tintedTransmission, reflected, clamp(fresnel, 0.0, 1.0));
    FragColor = vec4(color, clamp(uOpacity, 0.0, 1.0));
}
