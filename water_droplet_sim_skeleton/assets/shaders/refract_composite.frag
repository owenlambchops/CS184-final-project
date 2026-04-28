#version 330 core
in vec2 vUv;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uDropletNormal;
uniform sampler2D uEnvironmentMap;
uniform mat4 uInvViewRot;
uniform float uIor;
uniform float uRefractionScale;
uniform float uFresnelBias;
uniform float uFresnelScale;
uniform float uFresnelPower;
uniform float uSpecularPower;

const float kPi = 3.14159265359;

vec2 equirectUv(vec3 dir) {
    vec3 d = normalize(dir);
    float u = atan(d.z, d.x) / (2.0 * kPi) + 0.5;
    float v = asin(clamp(d.y, -1.0, 1.0)) / kPi + 0.5;
    return vec2(u, v);
}

vec3 toneMap(vec3 color) {
    vec3 mapped = color / (color + vec3(1.0));
    return pow(mapped, vec3(1.0 / 2.2));
}

void main() {
    vec3 scene = texture(uSceneColor, vUv).rgb;
    vec4 normalSample = texture(uDropletNormal, vUv);
    if (normalSample.a < 0.5) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    vec3 encN = normalSample.xyz;
    vec3 n = normalize(encN * 2.0 - 1.0);

    vec2 offset = n.xy * uRefractionScale;
    vec2 refractUv = clamp(vUv + offset, vec2(0.0), vec2(1.0));
    vec3 refracted = texture(uSceneColor, refractUv).rgb;

    float ndotv = clamp(n.z, 0.0, 1.0);
    float eta = max(uIor, 1.0001);
    float f0 = pow((1.0 - eta) / (1.0 + eta), 2.0);
    float fresnel = f0 + (1.0 - f0) * pow(1.0 - ndotv, uFresnelPower);
    fresnel = clamp(uFresnelBias + uFresnelScale * fresnel, 0.0, 1.0);

    vec3 reflectView = reflect(vec3(0.0, 0.0, -1.0), n);
    vec3 reflectWorld = normalize(mat3(uInvViewRot) * reflectView);
    vec3 envReflection = toneMap(texture(uEnvironmentMap, equirectUv(reflectWorld)).rgb);

    vec3 lightDir = normalize(vec3(-0.35, 0.45, 0.82));
    vec3 viewDir = vec3(0.0, 0.0, 1.0);
    vec3 halfDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(n, halfDir), 0.0), uSpecularPower);

    vec3 color = mix(refracted, envReflection, fresnel)
               + vec3(1.0) * spec * 0.45;

    FragColor = vec4(color, 1.0);
}
