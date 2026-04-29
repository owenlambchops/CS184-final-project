#version 330 core
in vec2 vUv;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uDropletNormal;
uniform sampler2D uEnvironmentMap;
uniform sampler2D uSceneDepth;
uniform sampler2D uDropletDepth;
uniform mat4 uInvProj;
uniform mat3 uInvViewRot;
uniform int uEnableThickness;
uniform float uIor;
uniform float uNearPlane;
uniform float uFarPlane;
uniform float uRefractionScale;
uniform float uFresnelBias;
uniform float uFresnelScale;
uniform float uFresnelPower;

const float kPi = 3.14159265359;
const float kMaxThickness = 1.0;
const float kAbsorptionStrength = 1.35;
const vec3 kAbsorptionColor = vec3(0.65, 0.24, 0.10);

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

float linearizeDepth(float depth) {
    float z = depth * 2.0 - 1.0;
    return (2.0 * uNearPlane * uFarPlane) /
           (uFarPlane + uNearPlane - z * (uFarPlane - uNearPlane));
}

float estimateThickness() {
    float sceneDepth = texture(uSceneDepth, vUv).r;
    float dropletDepth = texture(uDropletDepth, vUv).r;
    if (sceneDepth >= 0.9999 || dropletDepth >= 0.9999) {
        return 0.0;
    }

    float sceneLinear = linearizeDepth(sceneDepth);
    float dropletLinear = linearizeDepth(dropletDepth);
    return clamp(sceneLinear - dropletLinear, 0.0, kMaxThickness);
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
    vec2 ndc = vUv * 2.0 - 1.0;
    vec4 view = uInvProj * vec4(ndc, 1.0, 1.0);
    vec3 viewDir = normalize(view.xyz / view.w);

    vec2 offset = n.xy * uRefractionScale;
    vec2 refractUv = clamp(vUv + offset, vec2(0.0), vec2(1.0));
    vec3 refracted = texture(uSceneColor, refractUv).rgb;
    if (uEnableThickness != 0) {
        float thickness = estimateThickness();
        vec3 attenuation = exp(-kAbsorptionColor * thickness * kAbsorptionStrength);
        refracted *= attenuation;
    }

    float ndotv = clamp(dot(n, -viewDir), 0.0, 1.0);
    float eta = max(uIor, 1.0001);
    float f0 = pow((1.0 - eta) / (1.0 + eta), 2.0);
    float fresnel = f0 + (1.0 - f0) * pow(1.0 - ndotv, uFresnelPower);
    fresnel = clamp(uFresnelBias + uFresnelScale * fresnel, 0.0, 1.0);

    vec3 reflectView = reflect(viewDir, n);
    vec3 reflectWorld = normalize(uInvViewRot * reflectView);
    vec3 envReflection = toneMap(texture(uEnvironmentMap, equirectUv(reflectWorld)).rgb);

    vec3 color = mix(refracted, envReflection, fresnel);

    FragColor = vec4(color, 1.0);
}
