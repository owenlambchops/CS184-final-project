#version 330 core

in float vIntensity;
out vec4 FragColor;

void main() {
    vec2 p = gl_PointCoord * 2.0 - vec2(1.0);
    float r2 = dot(p, p);
    if (r2 > 1.0 || vIntensity <= 0.0) {
        discard;
    }

    float weight = exp(-3.0 * r2);
    FragColor = vec4(vec3(vIntensity * weight), 1.0);
}
