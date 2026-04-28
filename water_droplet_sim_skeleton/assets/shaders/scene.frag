#version 330 core
in vec3 vWorldNormal;
in vec3 vWorldPos;
out vec4 FragColor;

void main() {
    vec3 n = normalize(vWorldNormal);
    vec3 lightDir = normalize(vec3(0.4, 0.8, 0.2));
    float shade = 0.55 + 0.45 * max(dot(n, lightDir), 0.0);

    vec2 p = vWorldPos.xz;
    vec2 checkerCell = floor(p * 3.0);
    float checker = mod(checkerCell.x + checkerCell.y, 2.0);
    vec3 base = mix(vec3(0.14, 0.16, 0.17), vec3(0.72, 0.76, 0.72), checker);

    vec2 gridCoord = p * 12.0;
    vec2 gridDeriv = max(fwidth(gridCoord), vec2(1e-4));
    vec2 grid = abs(fract(gridCoord - 0.5) - 0.5) / gridDeriv;
    float gridLine = 1.0 - min(min(grid.x, grid.y), 1.0);

    vec3 color = mix(base, vec3(0.05, 0.45, 0.95), gridLine * 0.85);
    FragColor = vec4(color * shade, 1.0);
}
