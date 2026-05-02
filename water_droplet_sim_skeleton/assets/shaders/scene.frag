#version 330 core
in vec3 vWorldNormal;
in vec3 vWorldPos;
out vec4 FragColor;

void main() {
    vec3 n = normalize(vWorldNormal);
    float shade = max(dot(n, normalize(vec3(1.0, 1.0, 1.0))), 0.0);
    FragColor = vec4(vec3(shade), 1.0);
}
