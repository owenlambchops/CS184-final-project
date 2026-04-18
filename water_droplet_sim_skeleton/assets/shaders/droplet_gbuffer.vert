#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vWorldNormal;

void main() {
    vWorldPos = aPosition;
    vWorldNormal = normalize(aNormal);
    gl_Position = uProj * uView * vec4(aPosition, 1.0);
}
