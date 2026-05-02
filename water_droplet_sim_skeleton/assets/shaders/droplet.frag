#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNormal;
in float vDebugValue;

uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform int uDebugColorMode;

out vec4 FragColor;

void main() {
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-uLightDir);
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 H = normalize(L + V);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), 64.0);

    vec3 base = vec3(0.3, 0.6, 0.9);
    if (uDebugColorMode != 0) {
        float t = clamp(vDebugValue, 0.0, 1.0);
        vec3 cold;
        vec3 hot;

        if (uDebugColorMode == 1) {          // edge length
            cold = vec3(0.08, 0.20, 0.95);
            hot  = vec3(0.95, 0.15, 0.10);
        } else if (uDebugColorMode == 2) {   // speed
            cold = vec3(0.05, 0.55, 0.10);
            hot  = vec3(0.98, 0.95, 0.10);
        } else {                             // curvature
            cold = vec3(0.20, 0.10, 0.55);
            hot  = vec3(0.95, 0.35, 0.95);
        }

        FragColor = vec4(mix(cold, hot, t), 1.0);
        return;
    }
    vec3 color = 0.15 * base + 0.65 * diff * base + 0.8 * spec * vec3(1.0);
    FragColor = vec4(color, 1.0);
}
