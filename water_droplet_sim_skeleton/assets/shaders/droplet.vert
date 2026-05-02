#version 330 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aDebugValue;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vWorldNormal;
out float vDebugValue;

void main() {
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorldPos = world.xyz;
    vWorldNormal = mat3(transpose(inverse(uModel))) * aNormal;
    vDebugValue = aDebugValue;
    gl_Position = uProj * uView * world;
}
