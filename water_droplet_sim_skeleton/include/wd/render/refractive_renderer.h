#pragma once
#include "wd/core/scene.h"
#include "wd/render/droplet_gpu_cache.h"
#include "wd/render/shader.h"

namespace wd {

class Camera {
public:
    Eigen::Matrix4d view() const { return Eigen::Matrix4d::Identity(); }
    Eigen::Matrix4d proj(double) const { return Eigen::Matrix4d::Identity(); }
    Vec3 position() const { return position_; }
    void setPosition(const Vec3& p) { position_ = p; }

private:
    Vec3 position_ = Vec3(0.0, 1.0, 3.0);
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
    void releaseResources();
    void renderBasicScene(const Scene& scene, const Camera& camera);

    void renderSceneColorDepth(const Scene& scene, const Camera& camera);
    void renderDropletGBuffer(const Scene& scene, const Camera& camera);
    void compositeDroplets(const Scene& scene, const Camera& camera, const RenderParams& params);

    DropletGpuCache dropletCache_;
    Shader sceneShader_;
    Shader dropletShader_;
    unsigned int planeVao_ = 0;
    unsigned int planeVbo_ = 0;
    unsigned int planeEbo_ = 0;
    int planeIndexCount_ = 0;
    int width_ = 0;
    int height_ = 0;
};

} // namespace wd
