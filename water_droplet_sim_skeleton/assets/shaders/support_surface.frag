#version 330 core
in vec3 vWorldNormal;
out vec4 FragColor;

void main() {
    vec3 n = normalize(vWorldNormal);
    vec3 lightDir = normalize(vec3(0.4, 0.8, 0.2));
    float shade = 0.62 + 0.38 * max(dot(n, lightDir), 0.0);
    vec3 base = vec3(0.34, 0.36, 0.34);
    FragColor = vec4(base * shade, 1.0);
}
