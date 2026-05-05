#version 330 core
in vec3 vWorldPosition;
in vec3 vWorldNormal;
out vec4 FragColor;

uniform sampler2D uCausticMap;
uniform sampler2D uDropletShadowDepth;
uniform sampler2D uEnvironmentMap;

uniform int uEnableCaustics;
uniform int uEnableShadows;
uniform vec3 uLightDir;
uniform mat4 uLightViewProj;
uniform vec3 uPlaneOrigin;
uniform vec3 uPlaneTangentU;
uniform vec3 uPlaneTangentV;
uniform float uPlaneSideLength;

vec2 sampleSphericalMap(vec3 v) {
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= vec2(0.1591, 0.3183);
    uv += 0.5;
    return uv;
}

float sampleDropletShadow(vec2 uv, float fragDepth, float bias) {
    float shadowSum = 0.0;
    vec2 texel = 1.0 / vec2(textureSize(uDropletShadowDepth, 0));
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            vec2 offset = vec2(float(x), float(y)) * texel;
            float dropletDepth = texture(uDropletShadowDepth, uv + offset).r;
            shadowSum += (dropletDepth + bias < fragDepth) ? 1.0 : 0.0;
        }
    }
    return shadowSum / 9.0;
}

void main() {
    vec3 n = normalize(vWorldNormal);

    // FIX: Use uLightDir directly so it points towards the light source
    vec3 rawLight = uLightDir; 
    vec3 lightDir = length(rawLight) > 1e-6 ? normalize(rawLight) : vec3(0.0, 1.0, 0.0);
    float lambert = max(dot(n, lightDir), 0.0);
    float shadow = 1.0;
    
    if (uEnableShadows != 0) {
        vec4 lightClip = uLightViewProj * vec4(vWorldPosition, 1.0);
        vec3 lightNdc = lightClip.xyz / max(lightClip.w, 1e-8);
        vec2 shadowUv = lightNdc.xy * 0.5 + vec2(0.5);
        float fragDepth = lightNdc.z * 0.5 + 0.5;

        if (all(greaterThanEqual(shadowUv, vec2(0.0))) && all(lessThanEqual(shadowUv, vec2(1.0)))) {
            float depthBias = 0.0015;
            float shadowFactor = sampleDropletShadow(shadowUv, fragDepth, depthBias);
            shadow = mix(1.0, 0.18, shadowFactor);
        }
    }

    vec3 envColor = texture(uEnvironmentMap, sampleSphericalMap(n)).rgb;
    
    vec3 diffuseAmbient = vec3(1.0); 

    vec3 color = diffuseAmbient * (0.22 + 0.78 * lambert * shadow);

    if (uEnableCaustics != 0 && uPlaneSideLength > 0.0) {
        vec3 local = vWorldPosition - uPlaneOrigin;
        vec2 causticUv = vec2(dot(local, uPlaneTangentU), dot(local, uPlaneTangentV)) / uPlaneSideLength + vec2(0.5);
        if (all(greaterThanEqual(causticUv, vec2(0.0))) && all(lessThanEqual(causticUv, vec2(1.0)))) {
            float caustic = texture(uCausticMap, causticUv).r;
            color += caustic * vec3(1.0, 0.94, 0.72);
        }
    }
    
    FragColor = vec4(color, 1.0);
}