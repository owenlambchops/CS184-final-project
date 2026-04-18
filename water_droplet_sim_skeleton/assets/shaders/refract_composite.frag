#version 330 core
in vec2 vUv;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uDropletNormal;
uniform float uRefractionScale;
uniform float uFresnelBias;
uniform float uFresnelScale;
uniform float uFresnelPower;

void main() {
    vec3 encN = texture(uDropletNormal, vUv).xyz;
    vec3 n = normalize(encN * 2.0 - 1.0);

    vec2 offset = n.xy * uRefractionScale;
    vec3 refracted = texture(uSceneColor, vUv + offset).rgb;

    float ndotv = clamp(n.z, 0.0, 1.0);
    float fresnel = uFresnelBias + uFresnelScale * pow(1.0 - ndotv, uFresnelPower);

    FragColor = vec4(refracted + vec3(fresnel) * 0.15, 1.0);
}
