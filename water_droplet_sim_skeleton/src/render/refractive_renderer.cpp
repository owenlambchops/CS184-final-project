#include "wd/render/refractive_renderer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

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

struct CameraMatrices {
    glm::vec3 eye;
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 model;
};

CameraMatrices buildCameraMatrices(const Camera& camera, int width, int height) {
    CameraMatrices matrices{};
    matrices.eye = toGlm(camera.position());
    matrices.view = glm::lookAt(matrices.eye, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    matrices.proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
    matrices.model = glm::mat4(1.0f);
    return matrices;
}

const char* framebufferStatusName(unsigned int status) {
    switch (status) {
    case GL_FRAMEBUFFER_COMPLETE: return "GL_FRAMEBUFFER_COMPLETE";
    case GL_FRAMEBUFFER_UNDEFINED: return "GL_FRAMEBUFFER_UNDEFINED";
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT: return "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT: return "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER: return "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER: return "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
    case GL_FRAMEBUFFER_UNSUPPORTED: return "GL_FRAMEBUFFER_UNSUPPORTED";
    case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE: return "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
    case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS: return "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
    default: return "unknown framebuffer status";
    }
}

void configureColorTexture(unsigned int texture,
                           int width,
                           int height,
                           int internalFormat,
                           unsigned int format,
                           unsigned int type) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, width, height, 0, format, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void configureDepthTexture(unsigned int texture, int width, int height) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_DEPTH_COMPONENT24,
                 width,
                 height,
                 0,
                 GL_DEPTH_COMPONENT,
                 GL_UNSIGNED_INT,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
}

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
    if (sceneShader_.id() != 0 || dropletGBufferShader_.id() != 0 || compositeShader_.id() != 0) {
        if (!createRenderTargets()) {
            std::cerr << "Failed to resize renderer FBO attachments.\n";
        }
    }
}

RenderStats RefractiveRenderer::render(const Scene& scene, const Camera& camera, const RenderParams& params) {
    using Clock = std::chrono::high_resolution_clock;
    auto start = Clock::now();

    if (width_ > 0 && height_ > 0) {
        dropletCache_.sync(scene.droplets());

        renderSceneColorDepth(scene, camera);
        renderDropletGBuffer(scene, camera);
        compositeDroplets(scene, camera, params);
    }

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

    if (!dropletGBufferShader_.load("assets/shaders/droplet_gbuffer.vert",
                                    "assets/shaders/droplet_gbuffer.frag")) {
        return false;
    }

    if (!compositeShader_.load("assets/shaders/refract_composite.vert",
                               "assets/shaders/refract_composite.frag")) {
        return false;
    }

    initializePlaneMesh();
    glGenVertexArrays(1, &fullscreenVao_);

    return planeVao_ != 0 && planeEbo_ != 0 && fullscreenVao_ != 0 && createRenderTargets();
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

bool RefractiveRenderer::createRenderTargets() {
    releaseRenderTargets();

    const int targetWidth = std::max(width_, 1);
    const int targetHeight = std::max(height_, 1);

    glGenFramebuffers(1, &sceneFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);

    glGenTextures(1, &sceneColorTex_);
    configureColorTexture(sceneColorTex_,
                          targetWidth,
                          targetHeight,
                          GL_RGBA8,
                          GL_RGBA,
                          GL_UNSIGNED_BYTE);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           sceneColorTex_,
                           0);

    glGenTextures(1, &sceneDepthTex_);
    configureDepthTexture(sceneDepthTex_, targetWidth, targetHeight);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D,
                           sceneDepthTex_,
                           0);

    const std::array<unsigned int, 1> sceneDrawBuffers = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(static_cast<int>(sceneDrawBuffers.size()), sceneDrawBuffers.data());
    if (!checkFramebufferComplete("sceneFbo")) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        releaseRenderTargets();
        return false;
    }

    glGenFramebuffers(1, &dropletFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, dropletFbo_);

    glGenTextures(1, &dropletNormalTex_);
    configureColorTexture(dropletNormalTex_,
                          targetWidth,
                          targetHeight,
                          GL_RGBA16F,
                          GL_RGBA,
                          GL_FLOAT);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D,
                           dropletNormalTex_,
                           0);

    glGenTextures(1, &dropletThicknessTex_);
    configureColorTexture(dropletThicknessTex_,
                          targetWidth,
                          targetHeight,
                          GL_RGBA16F,
                          GL_RGBA,
                          GL_FLOAT);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_COLOR_ATTACHMENT1,
                           GL_TEXTURE_2D,
                           dropletThicknessTex_,
                           0);

    glGenTextures(1, &dropletDepthTex_);
    configureDepthTexture(dropletDepthTex_, targetWidth, targetHeight);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D,
                           dropletDepthTex_,
                           0);

    const std::array<unsigned int, 2> dropletDrawBuffers = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
    };
    glDrawBuffers(static_cast<int>(dropletDrawBuffers.size()), dropletDrawBuffers.data());
    if (!checkFramebufferComplete("dropletFbo")) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        releaseRenderTargets();
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void RefractiveRenderer::releaseRenderTargets() {
    if (dropletDepthTex_ != 0) glDeleteTextures(1, &dropletDepthTex_);
    if (dropletThicknessTex_ != 0) glDeleteTextures(1, &dropletThicknessTex_);
    if (dropletNormalTex_ != 0) glDeleteTextures(1, &dropletNormalTex_);
    if (dropletFbo_ != 0) glDeleteFramebuffers(1, &dropletFbo_);

    if (sceneDepthTex_ != 0) glDeleteTextures(1, &sceneDepthTex_);
    if (sceneColorTex_ != 0) glDeleteTextures(1, &sceneColorTex_);
    if (sceneFbo_ != 0) glDeleteFramebuffers(1, &sceneFbo_);

    dropletDepthTex_ = 0;
    dropletThicknessTex_ = 0;
    dropletNormalTex_ = 0;
    dropletFbo_ = 0;

    sceneDepthTex_ = 0;
    sceneColorTex_ = 0;
    sceneFbo_ = 0;
}

bool RefractiveRenderer::checkFramebufferComplete(const char* name) const {
    const unsigned int status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status == GL_FRAMEBUFFER_COMPLETE) return true;

    std::cerr << name << " is incomplete: " << framebufferStatusName(status)
              << " (" << status << ")\n";
    return false;
}

void RefractiveRenderer::releaseResources() {
    releaseRenderTargets();
    dropletCache_.clear();

    if (planeEbo_ != 0) glDeleteBuffers(1, &planeEbo_);
    if (planeVbo_ != 0) glDeleteBuffers(1, &planeVbo_);
    if (planeVao_ != 0) glDeleteVertexArrays(1, &planeVao_);
    if (fullscreenVao_ != 0) glDeleteVertexArrays(1, &fullscreenVao_);

    planeVao_ = 0;
    planeVbo_ = 0;
    planeEbo_ = 0;
    fullscreenVao_ = 0;
    planeIndexCount_ = 0;

    sceneShader_.reset();
    dropletGBufferShader_.reset();
    compositeShader_.reset();
}

void RefractiveRenderer::renderSceneColorDepth(const Scene&, const Camera& camera) {
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
    glViewport(0, 0, std::max(width_, 1), std::max(height_, 1));
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);

    if (sceneShader_.id() != 0 && planeVao_ != 0) {
        sceneShader_.use();
        sceneShader_.setMat4("uModel", matrices.model);
        sceneShader_.setMat4("uView", matrices.view);
        sceneShader_.setMat4("uProj", matrices.proj);

        glBindVertexArray(planeVao_);
        glDrawElements(GL_TRIANGLES, planeIndexCount_, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    glUseProgram(0);
}

void RefractiveRenderer::renderDropletGBuffer(const Scene& scene, const Camera& camera) {
    glBindFramebuffer(GL_FRAMEBUFFER, dropletFbo_);
    glViewport(0, 0, std::max(width_, 1), std::max(height_, 1));
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    const std::array<float, 4> normalClear = {0.5f, 0.5f, 1.0f, 0.0f};
    const std::array<float, 4> thicknessClear = {0.0f, 0.0f, 0.0f, 0.0f};
    constexpr float depthClear = 1.0f;
    glClearBufferfv(GL_COLOR, 0, normalClear.data());
    glClearBufferfv(GL_COLOR, 1, thicknessClear.data());
    glClearBufferfv(GL_DEPTH, 0, &depthClear);

    const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);

    if (dropletGBufferShader_.id() != 0) {
        dropletGBufferShader_.use();
        dropletGBufferShader_.setMat4("uView", matrices.view);
        dropletGBufferShader_.setMat4("uProj", matrices.proj);

        for (const auto& droplet : scene.droplets()) {
            dropletCache_.drawDroplet(droplet->id());
        }
    }

    glUseProgram(0);
}

void RefractiveRenderer::compositeDroplets(const Scene&, const Camera&, const RenderParams& params) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width_, height_);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    if (compositeShader_.id() != 0 && fullscreenVao_ != 0) {
        compositeShader_.use();
        compositeShader_.setInt("uSceneColor", 0);
        compositeShader_.setInt("uDropletNormal", 1);
        compositeShader_.setFloat("uIor", static_cast<float>(params.ior));
        compositeShader_.setFloat("uRefractionScale", static_cast<float>(params.refractionScale));
        compositeShader_.setFloat("uFresnelBias", static_cast<float>(params.fresnelBias));
        compositeShader_.setFloat("uFresnelScale", static_cast<float>(params.fresnelScale));
        compositeShader_.setFloat("uFresnelPower", static_cast<float>(params.fresnelPower));
        compositeShader_.setFloat("uSpecularPower", static_cast<float>(params.specularPower));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sceneColorTex_);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, dropletNormalTex_);

        glBindVertexArray(fullscreenVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(0);
}

} // namespace wd
