#version 330 core
in vec3 vWorldNormal;
in vec3 vWorldPos;
out vec4 FragColor;

void main() {
    vec3 n = normalize(vWorldNormal);
    float shade = 0.4 + 0.6 * max(dot(n, normalize(vec3(0.4, 0.8, 0.2))), 0.0);
    FragColor = vec4(vec3(shade), 1.0);
}
