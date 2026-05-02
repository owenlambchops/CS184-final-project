#version 330 core
in vec3 vViewNormal;

layout(location = 0) out vec4 outNormal;

void main() {
    outNormal = vec4(normalize(vViewNormal) * 0.5 + 0.5, 1.0);
}
