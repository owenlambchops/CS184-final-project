#pragma once
#include "wd/core/scene.h"
#include "wd/render/droplet_gpu_cache.h"
#include "wd/render/shader.h"
#include <glm/fwd.hpp>

namespace wd {

class Camera {
public:
    glm::mat4 view() const;
    glm::mat4 proj(double aspect) const;

    Vec3 position() const { return position_; }
    void setPosition(const Vec3& p) { position_ = p; }
    void move(const Vec3& delta) { position_ += delta; }

    Vec3 forward() const;
    Vec3 right() const;
    Vec3 up() const;
    double fovYRad() const;

    void rotate(double yawDeltaRad, double pitchDeltaRad);
    void lookAt(const Vec3& target);

private:
    Vec3 position_ = Vec3(0.0, 1.0, 3.0);
    double yawRad_ = 0.0;
    double pitchRad_ = 0.0;
    double fovYDeg_ = 45.0;
};

class RefractiveRenderer {
public:
    ~RefractiveRenderer();

    bool initialize(int width, int height);
    void resize(int width, int height);

    RenderStats render(const Scene& scene, const Camera& camera, const RenderParams& params);

private:
    bool initializeBasicResources();
    void initializePlaneMesh();
    bool createRenderTargets();
    bool initializeEnvironmentMap();
    void releaseEnvironmentMap();
    void releaseRenderTargets();
    bool checkFramebufferComplete(const char* name) const;
    void releaseResources();

    void renderEnvironmentBackground(const Camera& camera);
    void renderSceneColorDepth(const Scene& scene, const Camera& camera);
    void renderDropletGBuffer(const Scene& scene, const Camera& camera);
    void compositeDroplets(const Scene& scene, const Camera& camera, const RenderParams& params);

    DropletGpuCache dropletCache_;
    Shader supportSurfaceShader_;
    Shader backgroundShader_;
    Shader dropletGBufferShader_;
    Shader compositeShader_;
    unsigned int planeVao_ = 0;
    unsigned int planeVbo_ = 0;
    unsigned int planeEbo_ = 0;
    unsigned int fullscreenVao_ = 0;

    unsigned int sceneFbo_ = 0;
    unsigned int sceneColorTex_ = 0;
    unsigned int sceneDepthTex_ = 0;

    unsigned int dropletFbo_ = 0;
    unsigned int dropletNormalTex_ = 0;
    unsigned int dropletThicknessTex_ = 0;
    unsigned int dropletDepthTex_ = 0;

    unsigned int environmentTex_ = 0;

    int planeIndexCount_ = 0;
    int width_ = 0;
    int height_ = 0;
};

} // namespace wd
