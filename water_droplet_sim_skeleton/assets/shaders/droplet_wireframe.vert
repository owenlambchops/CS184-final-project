#version 330 core
layout(location = 0) in vec3 aPosition;

uniform mat4 uView;
uniform mat4 uProj;

void main() {
    gl_Position = uProj * uView * vec4(aPosition, 1.0);
}
