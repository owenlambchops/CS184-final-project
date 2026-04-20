#include "wd/render/refractive_renderer.h"

#include <array>
#include <chrono>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace wd {

namespace {

glm::vec3 toGlm(const Vec3& v) {
    return glm::vec3(static_cast<float>(v.x()),
                     static_cast<float>(v.y()),
                     static_cast<float>(v.z()));
}

constexpr std::array<float, 24> kPlaneVertices = {
    -3.0f, 0.0f, -3.0f, 0.0f, 1.0f, 0.0f,
     3.0f, 0.0f, -3.0f, 0.0f, 1.0f, 0.0f,
     3.0f, 0.0f,  3.0f, 0.0f, 1.0f, 0.0f,
    -3.0f, 0.0f,  3.0f, 0.0f, 1.0f, 0.0f,
};

constexpr std::array<unsigned int, 6> kPlaneIndices = {0, 1, 2, 0, 2, 3};

const glm::vec3 kLightDir = glm::normalize(glm::vec3(0.4f, 0.8f, 0.2f));

} // namespace

RefractiveRenderer::~RefractiveRenderer() {
    releaseResources();
}

bool RefractiveRenderer::initialize(int width, int height) {
    width_ = width;
    height_ = height;
    return initializeBasicResources();
}

void RefractiveRenderer::resize(int width, int height) {
    width_ = width;
    height_ = height;
    // TODO: resize FBO attachments.
}

RenderStats RefractiveRenderer::render(const Scene& scene, const Camera& camera, const RenderParams& params) {
    using Clock = std::chrono::high_resolution_clock;
    auto start = Clock::now();

    static_cast<void>(params);

    glViewport(0, 0, width_, height_);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderBasicScene(scene, camera);

    auto end = Clock::now();
    RenderStats stats{};
    stats.frameWidth = width_;
    stats.frameHeight = height_;
    stats.renderMs = std::chrono::duration<double, std::milli>(end - start).count();
    return stats;
}

bool RefractiveRenderer::initializeBasicResources() {
    releaseResources();

    if (!sceneShader_.load("assets/shaders/scene.vert", "assets/shaders/scene.frag")) {
        return false;
    }

    if (!dropletShader_.load("assets/shaders/droplet.vert", "assets/shaders/droplet.frag")) {
        return false;
    }

    initializePlaneMesh();
    return planeVao_ != 0 && planeEbo_ != 0;
}

void RefractiveRenderer::initializePlaneMesh() {
    planeIndexCount_ = static_cast<int>(kPlaneIndices.size());

    glGenVertexArrays(1, &planeVao_);
    glGenBuffers(1, &planeVbo_);
    glGenBuffers(1, &planeEbo_);

    glBindVertexArray(planeVao_);

    glBindBuffer(GL_ARRAY_BUFFER, planeVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(kPlaneVertices.size() * sizeof(float)),
                 kPlaneVertices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(kPlaneIndices.size() * sizeof(unsigned int)),
                 kPlaneIndices.data(),
                 GL_STATIC_DRAW);

    constexpr int stride = 6 * static_cast<int>(sizeof(float));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void RefractiveRenderer::releaseResources() {
    dropletCache_.clear();

    if (planeEbo_ != 0) glDeleteBuffers(1, &planeEbo_);
    if (planeVbo_ != 0) glDeleteBuffers(1, &planeVbo_);
    if (planeVao_ != 0) glDeleteVertexArrays(1, &planeVao_);

    planeVao_ = 0;
    planeVbo_ = 0;
    planeEbo_ = 0;
    planeIndexCount_ = 0;

    sceneShader_.reset();
    dropletShader_.reset();
}

void RefractiveRenderer::renderBasicScene(const Scene& scene, const Camera& camera) {
    const glm::vec3 eye = toGlm(camera.position());
    const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const float aspect = height_ > 0 ? static_cast<float>(width_) / static_cast<float>(height_) : 1.0f;
    const glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
    const glm::mat4 model(1.0f);

    if (sceneShader_.id() != 0 && planeVao_ != 0) {
        sceneShader_.use();
        sceneShader_.setMat4("uModel", model);
        sceneShader_.setMat4("uView", view);
        sceneShader_.setMat4("uProj", proj);

        glBindVertexArray(planeVao_);
        glDrawElements(GL_TRIANGLES, planeIndexCount_, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    if (dropletShader_.id() != 0) {
        dropletCache_.sync(scene.droplets());

        dropletShader_.use();
        dropletShader_.setMat4("uModel", model);
        dropletShader_.setMat4("uView", view);
        dropletShader_.setMat4("uProj", proj);
        dropletShader_.setVec3("uLightDir", kLightDir);
        dropletShader_.setVec3("uViewPos", eye);

        for (const auto& droplet : scene.droplets()) {
            dropletCache_.drawDroplet(droplet->id());
        }
    }

    glUseProgram(0);
}

void RefractiveRenderer::renderSceneColorDepth(const Scene&, const Camera&) {
    // TODO: draw background/support surface into scene color and depth textures.
}

void RefractiveRenderer::renderDropletGBuffer(const Scene&, const Camera&) {
    // TODO: draw droplet normals, depth, and thickness.
}

void RefractiveRenderer::compositeDroplets(const Scene&, const Camera&, const RenderParams&) {
    // TODO: full-screen composite: scene color + normal-based refraction + Fresnel/specular.
}

} // namespace wd
