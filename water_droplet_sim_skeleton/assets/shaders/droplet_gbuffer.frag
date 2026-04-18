#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNormal;

layout(location = 0) out vec4 outNormal;
layout(location = 1) out vec4 outThickness;

void main() {
    outNormal = vec4(normalize(vWorldNormal) * 0.5 + 0.5, 1.0);
    outThickness = vec4(vec3(1.0), 1.0); // TODO: replace with real thickness accumulation.
}
