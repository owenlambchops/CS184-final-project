#version 330 core

uniform sampler2D uLightNormal;
uniform sampler2D uLightFrontDepth;
uniform sampler2D uLightBackDepth;
uniform mat4 uInvLightView;
uniform mat4 uInvLightProj;
uniform vec3 uLightRayDir;
uniform vec3 uPlaneOrigin;
uniform vec3 uPlaneNormal;
uniform vec3 uPlaneTangentU;
uniform vec3 uPlaneTangentV;
uniform float uPlaneSideLength;
uniform float uIor;
uniform float uMaxThickness;
uniform float uCausticStrength;
uniform float uCausticPointSize;
uniform int uMapSize;

out float vIntensity;

vec3 reconstructWorld(vec2 uv, float depth) {
    vec4 clip = vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 view = uInvLightProj * clip;
    float invW = 1.0;
    if (abs(view.w) > 1e-6) {
        invW = 1.0 / view.w;
    }
    view *= invW;
    vec4 world = uInvLightView * view;
    return world.xyz;
}

void rejectPhoton() {
    vIntensity = 0.0;
    gl_Position = vec4(2.0, 2.0, 0.0, 1.0);
    gl_PointSize = 1.0;
}

void main() {
    int x = gl_VertexID % uMapSize;
    int y = gl_VertexID / uMapSize;
    vec2 lightUv = (vec2(x, y) + vec2(0.5)) / float(uMapSize);

    vec4 normalSample = texture(uLightNormal, lightUv);
    float frontDepth = texture(uLightFrontDepth, lightUv).r;
    float backDepth = texture(uLightBackDepth, lightUv).r;
    if (normalSample.a <= 0.0 || frontDepth >= 1.0 || backDepth >= 1.0) {
        rejectPhoton();
        return;
    }

    vec3 frontWorld = reconstructWorld(lightUv, frontDepth);
    vec3 backWorld = reconstructWorld(lightUv, backDepth);
    float thickness = length(backWorld - frontWorld);
    if (thickness <= 1e-5) {
        rejectPhoton();
        return;
    }

    vec3 normal = normalize(mat3(uInvLightView) * (normalSample.xyz * 2.0 - vec3(1.0)));
    vec3 incident = normalize(uLightRayDir);
    if (dot(incident, normal) > 0.0) {
        normal = -normal;
    }

    vec3 refracted = refract(incident, normal, 1.0 / max(uIor, 1.0001));
    if (dot(refracted, refracted) <= 1e-8) {
        rejectPhoton();
        return;
    }

    float denom = dot(refracted, uPlaneNormal);
    if (abs(denom) <= 1e-5) {
        rejectPhoton();
        return;
    }

    float t = dot(uPlaneOrigin - frontWorld, uPlaneNormal) / denom;
    if (t <= 0.0) {
        rejectPhoton();
        return;
    }

    vec3 hitWorld = frontWorld + refracted * t;
    vec3 planeLocal = hitWorld - uPlaneOrigin;
    vec2 planeUv = vec2(dot(planeLocal, uPlaneTangentU), dot(planeLocal, uPlaneTangentV)) /
                   uPlaneSideLength + vec2(0.5);
    if (any(lessThan(planeUv, vec2(0.0))) || any(greaterThan(planeUv, vec2(1.0)))) {
        rejectPhoton();
        return;
    }

    float incidence = clamp(dot(-incident, normal), 0.0, 1.0);
    float thicknessWeight = clamp(thickness / max(uMaxThickness, 1e-4), 0.0, 1.0);
    vIntensity = uCausticStrength * incidence * thicknessWeight;
    gl_Position = vec4(planeUv * 2.0 - 1.0, 0.0, 1.0);
    gl_PointSize = uCausticPointSize;
}
