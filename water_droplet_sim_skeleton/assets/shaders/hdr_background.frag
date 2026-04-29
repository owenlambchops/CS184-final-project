#version 330 core
in vec2 vUv;
out vec4 FragColor;

uniform sampler2D uEnvironmentMap;
uniform mat4 uInvProj;
uniform mat3 uInvViewRot;

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
    vec2 ndc = vUv * 2.0 - 1.0;
    vec4 view = uInvProj * vec4(ndc, 1.0, 1.0);
    vec3 viewDir = normalize(view.xyz / view.w);
    vec3 worldDir = normalize(uInvViewRot * viewDir);

    vec3 hdr = texture(uEnvironmentMap, equirectUv(worldDir)).rgb;
    FragColor = vec4(toneMap(hdr), 1.0);
}
