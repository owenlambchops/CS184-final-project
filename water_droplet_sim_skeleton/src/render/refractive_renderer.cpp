#include "wd/render/refractive_renderer.h"
#include <chrono>

namespace wd {

void RefractiveRenderer::initialize(int width, int height) {
    width_ = width;
    height_ = height;
    // TODO: create FBOs and textures.
}

void RefractiveRenderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    // TODO: resize FBO attachments.
}

RenderStats RefractiveRenderer::render(const Scene& scene, const Camera& camera, const RenderParams& params) {
    using Clock = std::chrono::high_resolution_clock;
    auto start = Clock::now();

    dropletCache_.sync(scene.droplets());
    renderSceneColorDepth(scene, camera);
    renderDropletGBuffer(scene, camera);
    compositeDroplets(scene, camera, params);

    auto end = Clock::now();
    RenderStats stats;
    stats.frameWidth = width_;
    stats.frameHeight = height_;
    stats.renderMs = std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}

void RefractiveRenderer::renderSceneColorDepth(const Scene&, const Camera&) {
    // TODO: draw background/support surface into scene color and depth textures.
}

void RefractiveRenderer::renderDropletGBuffer(const Scene& scene, const Camera&) {
    // TODO: draw droplet normals, depth, and thickness.
    for (const auto& d : scene.droplets()) {
        dropletCache_.drawDroplet(d->id());
    }
}

void RefractiveRenderer::compositeDroplets(const Scene&, const Camera&, const RenderParams&) {
    // TODO: full-screen composite: scene color + normal-based refraction + Fresnel/specular.
}

} // namespace wd
