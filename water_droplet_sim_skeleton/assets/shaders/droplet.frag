#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNormal;

uniform vec3 uLightDir;
uniform vec3 uViewPos;

out vec4 FragColor;

void main() {
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 H = normalize(L + V);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 64.0);

    vec3 base = vec3(0.3, 0.6, 0.9);
    vec3 color = 0.15 * base + 0.65 * diff * base + 0.8 * spec * vec3(1.0);
    FragColor = vec4(color, 1.0);
}
