#version 330 core
in vec2 vUv;
out vec4 FragColor;

uniform sampler2D uSceneColor;
uniform sampler2D uDropletNormal;
uniform sampler2D uEnvironmentMap;
uniform sampler2D uSceneDepth;
uniform sampler2D uDropletDepth;
uniform sampler2D uDropletBackDepth;
uniform sampler2D uCausticMap;
uniform mat4 uInvProj;
uniform mat3 uInvViewRot;
uniform int uEnableThickness;
uniform int uDebugView;
uniform float uIor;
uniform float uNearPlane;
uniform float uFarPlane;
uniform float uRefractionScale;
uniform float uFresnelBias;
uniform float uFresnelScale;
uniform float uFresnelPower;
uniform float uMaxThickness;
uniform float uDebugDepthRange;
uniform float uAbsorptionStrength;
uniform vec3 uAbsorptionColor;

const float kPi = 3.14159265359;
const int kDebugFinal = 0;
const int kDebugSceneColor = 1;
const int kDebugEnvironmentMap = 2;
const int kDebugSceneDepth = 3;
const int kDebugDropletDepth = 4;
const int kDebugDropletNormal = 5;
const int kDebugThickness = 6;
const int kDebugWireframe = 7;
const int kDebugCaustics = 7;

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
    float frontDepth = texture(uDropletDepth, vUv).r;
    float backDepth = texture(uDropletBackDepth, vUv).r;
    if (frontDepth >= 0.9999 || backDepth >= 0.9999) {
        return 0.0;
    }

    float frontLinear = linearizeDepth(frontDepth);
    float backLinear = linearizeDepth(backDepth);
    return clamp(backLinear - frontLinear, 0.0, uMaxThickness);
}

float visualizeSceneDepth(float depth) {
    if (depth >= 0.9999) {
        return 0.0;
    }
    return 1.0 - clamp(linearizeDepth(depth) / uDebugDepthRange, 0.0, 1.0);
}

float visualizeDropletDepth(float depth) {
    if (depth >= 0.9999) {
        return 0.0;
    }
    return 1.0 - clamp(linearizeDepth(depth) / uDebugDepthRange, 0.0, 1.0);
}

float wireframeFromDropletDepth() {
    vec2 texel = vec2(1.0) / vec2(textureSize(uDropletDepth, 0));
    float dC = texture(uDropletDepth, vUv).r;
    if (dC >= 0.9999) return 0.0;

    float dL = texture(uDropletDepth, vUv + vec2(-texel.x, 0.0)).r;
    float dR = texture(uDropletDepth, vUv + vec2( texel.x, 0.0)).r;
    float dU = texture(uDropletDepth, vUv + vec2(0.0, -texel.y)).r;
    float dD = texture(uDropletDepth, vUv + vec2(0.0,  texel.y)).r;

    float gx = abs(dR - dL);
    float gy = abs(dD - dU);
    float edge = gx + gy;
    return smoothstep(0.0008, 0.004, edge);
}

void main() {
    vec3 scene = texture(uSceneColor, vUv).rgb;
    vec2 ndc = vUv * 2.0 - 1.0;
    vec4 view = uInvProj * vec4(ndc, 1.0, 1.0);
    vec3 viewDir = normalize(view.xyz / view.w);

    if (uDebugView == kDebugSceneColor) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    if (uDebugView == kDebugEnvironmentMap) {
        vec3 worldDir = normalize(uInvViewRot * viewDir);
        vec3 environment = toneMap(texture(uEnvironmentMap, equirectUv(worldDir)).rgb);
        FragColor = vec4(environment, 1.0);
        return;
    }

    if (uDebugView == kDebugSceneDepth) {
        float depth = visualizeSceneDepth(texture(uSceneDepth, vUv).r);
        FragColor = vec4(vec3(depth), 1.0);
        return;
    }

    if (uDebugView == kDebugDropletDepth) {
        float depth = visualizeDropletDepth(texture(uDropletDepth, vUv).r);
        FragColor = vec4(vec3(depth), 1.0);
        return;
    }

    if (uDebugView == kDebugThickness) {
        float thickness = estimateThickness();
        FragColor = vec4(vec3(thickness), 1.0);
        return;
    }
    if (uDebugView == kDebugWireframe) {
        float wire = wireframeFromDropletDepth();
        vec3 edgeColor = vec3(0.15, 0.95, 0.85);
        FragColor = vec4(wire * edgeColor, 1.0);
        return;
    }

    if (uDebugView == kDebugCaustics) {
        float caustic = texture(uCausticMap, vUv).r;
        FragColor = vec4(vec3(caustic), 1.0);
        return;
    }

    vec4 normalSample = texture(uDropletNormal, vUv);
    if (uDebugView == kDebugDropletNormal) {
        FragColor = normalSample.a < 0.5 ? vec4(0.0, 0.0, 0.0, 1.0)
                                         : vec4(normalSample.xyz, 1.0);
        return;
    }

    if (normalSample.a < 0.5) {
        FragColor = vec4(scene, 1.0);
        return;
    }

    vec3 encN = normalSample.xyz;
    vec3 n = normalize(encN * 2.0 - 1.0);

    vec2 offset = n.xy * uRefractionScale;
    vec2 refractUv = clamp(vUv + offset, vec2(0.0), vec2(1.0));
    vec3 refracted = texture(uSceneColor, refractUv).rgb;
    if (uEnableThickness != 0) {
        float thickness = estimateThickness();
        vec3 attenuation = exp(-uAbsorptionColor * thickness * uAbsorptionStrength);
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
