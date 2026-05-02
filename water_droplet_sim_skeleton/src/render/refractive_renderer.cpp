#include "wd/render/refractive_renderer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace wd {

namespace {

glm::vec3 toGlm(const Vec3& v) {
    return glm::vec3(static_cast<float>(v.x()),
                     static_cast<float>(v.y()),
                     static_cast<float>(v.z()));
}

constexpr const char* kEnvironmentPath = "assets/static/suburban_garden_4k.hdr";
constexpr float kCameraNearPlane = 0.1f;
constexpr float kCameraFarPlane = 100.0f;

struct CameraMatrices {
    glm::vec3 eye;
    glm::mat4 view;
    glm::mat4 proj;
    glm::mat4 model;
};

CameraMatrices buildCameraMatrices(const Camera& camera, int width, int height) {
    CameraMatrices matrices{};
    matrices.eye = toGlm(camera.position());
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
    matrices.view = camera.view();
    matrices.proj = camera.proj(aspect);
    matrices.model = glm::mat4(1.0f);
    return matrices;
}

glm::mat3 inverseViewRotation(const glm::mat4& view) {
    return glm::mat3(glm::inverse(view));
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

Vec3 Camera::forward() const {
    const double cosPitch = std::cos(pitchRad_);
    return Vec3(cosPitch * std::cos(yawRad_),
                std::sin(pitchRad_),
                cosPitch * std::sin(yawRad_)).normalized();
}

Vec3 Camera::right() const {
    const Vec3 r = forward().cross(Vec3::UnitY());
    return r.norm() > 1e-8 ? r.normalized() : Vec3::UnitX();
}

Vec3 Camera::up() const {
    return right().cross(forward()).normalized();
}

double Camera::fovYRad() const {
    return fovYDeg_ * std::numbers::pi / 180.0;
}

glm::mat4 Camera::view() const {
    const glm::vec3 eye = toGlm(position_);
    return glm::lookAt(eye, eye + toGlm(forward()), toGlm(up()));
}

glm::mat4 Camera::proj(double aspect) const {
    const float safeAspect = static_cast<float>(std::max(aspect, 1e-6));
    return glm::perspective(static_cast<float>(fovYRad()), safeAspect, kCameraNearPlane, kCameraFarPlane);
}

void Camera::rotate(double yawDeltaRad, double pitchDeltaRad) {
    constexpr double maxPitch = 89.0 * std::numbers::pi / 180.0;
    yawRad_ += yawDeltaRad;
    pitchRad_ = std::clamp(pitchRad_ + pitchDeltaRad, -maxPitch, maxPitch);
}

void Camera::lookAt(const Vec3& target) {
    Vec3 dir = target - position_;
    if (dir.norm() <= 1e-8) return;

    dir.normalize();
    pitchRad_ = std::asin(std::clamp(dir.y(), -1.0, 1.0));
    yawRad_ = std::atan2(dir.z(), dir.x());
}

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
    if (supportSurfaceShader_.id() != 0 || dropletGBufferShader_.id() != 0 || compositeShader_.id() != 0) {
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
        renderDropletBackDepth(scene, camera);
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

    if (!supportSurfaceShader_.load("assets/shaders/support_surface.vert", "assets/shaders/support_surface.frag")) {
        return false;
    }

    if (!backgroundShader_.load("assets/shaders/hdr_background.vert",
                                "assets/shaders/hdr_background.frag")) {
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

    initializeSurfaceMeshBuffers();
    glGenVertexArrays(1, &fullscreenVao_);

    return surfaceVao_ != 0 && surfaceEbo_ != 0 && fullscreenVao_ != 0 &&
           initializeEnvironmentMap() && createRenderTargets();
}

void RefractiveRenderer::initializeSurfaceMeshBuffers() {
    glGenVertexArrays(1, &surfaceVao_);
    glGenBuffers(1, &surfaceVbo_);
    glGenBuffers(1, &surfaceEbo_);

    glBindVertexArray(surfaceVao_);

    glBindBuffer(GL_ARRAY_BUFFER, surfaceVbo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, surfaceEbo_);

    constexpr int stride = 6 * static_cast<int>(sizeof(float));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void RefractiveRenderer::syncSurfaceMesh(const ISurface& surface) {
    if (surfaceVao_ == 0 || surfaceVbo_ == 0 || surfaceEbo_ == 0) return;

    const SurfaceRenderMesh mesh = surface.buildRenderMesh();
    if (mesh.positions.rows() == 0 || mesh.faces.rows() == 0 ||
        mesh.positions.rows() != mesh.normals.rows()) {
        surfaceIndexCount_ = 0;
        return;
    }

    std::vector<float> vertices;
    vertices.reserve(static_cast<size_t>(mesh.positions.rows()) * 6);
    for (int i = 0; i < mesh.positions.rows(); ++i) {
        Vec3 p = mesh.positions.row(i).transpose();
        Vec3 n = mesh.normals.row(i).transpose();
        vertices.push_back(static_cast<float>(p.x()));
        vertices.push_back(static_cast<float>(p.y()));
        vertices.push_back(static_cast<float>(p.z()));
        vertices.push_back(static_cast<float>(n.x()));
        vertices.push_back(static_cast<float>(n.y()));
        vertices.push_back(static_cast<float>(n.z()));
    }

    std::vector<unsigned int> indices;
    indices.reserve(static_cast<size_t>(mesh.faces.rows()) * 3);
    for (int i = 0; i < mesh.faces.rows(); ++i) {
        indices.push_back(static_cast<unsigned int>(mesh.faces(i, 0)));
        indices.push_back(static_cast<unsigned int>(mesh.faces(i, 1)));
        indices.push_back(static_cast<unsigned int>(mesh.faces(i, 2)));
    }
    surfaceIndexCount_ = static_cast<int>(indices.size());

    glBindVertexArray(surfaceVao_);
    glBindBuffer(GL_ARRAY_BUFFER, surfaceVbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                 vertices.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, surfaceEbo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

bool RefractiveRenderer::initializeEnvironmentMap() {
    releaseEnvironmentMap();

    const std::filesystem::path hdrPath = kEnvironmentPath;
    int imageWidth = 0;
    int imageHeight = 0;
    int channelCount = 0;
    stbi_set_flip_vertically_on_load(true);
    float* pixels = stbi_loadf(hdrPath.string().c_str(), &imageWidth, &imageHeight, &channelCount, 3);
    if (pixels == nullptr) {
        std::cerr << "Failed to load HDR environment: " << std::filesystem::absolute(hdrPath)
                  << "\n" << stbi_failure_reason() << "\n";
        return false;
    }

    glGenTextures(1, &environmentTex_);
    glBindTexture(GL_TEXTURE_2D, environmentTex_);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB16F,
                 imageWidth,
                 imageHeight,
                 0,
                 GL_RGB,
                 GL_FLOAT,
                 pixels);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(pixels);
    return environmentTex_ != 0;
}

void RefractiveRenderer::releaseEnvironmentMap() {
    if (environmentTex_ != 0) glDeleteTextures(1, &environmentTex_);
    environmentTex_ = 0;
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

    glGenTextures(1, &dropletDepthTex_);
    configureDepthTexture(dropletDepthTex_, targetWidth, targetHeight);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D,
                           dropletDepthTex_,
                           0);

    const std::array<unsigned int, 1> dropletDrawBuffers = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(static_cast<int>(dropletDrawBuffers.size()), dropletDrawBuffers.data());
    if (!checkFramebufferComplete("dropletFbo")) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        releaseRenderTargets();
        return false;
    }

    glGenFramebuffers(1, &dropletBackDepthFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, dropletBackDepthFbo_);

    glGenTextures(1, &dropletBackDepthTex_);
    configureDepthTexture(dropletBackDepthTex_, targetWidth, targetHeight);
    glFramebufferTexture2D(GL_FRAMEBUFFER,
                           GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D,
                           dropletBackDepthTex_,
                           0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (!checkFramebufferComplete("dropletBackDepthFbo")) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        releaseRenderTargets();
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void RefractiveRenderer::releaseRenderTargets() {
    if (dropletBackDepthTex_ != 0) glDeleteTextures(1, &dropletBackDepthTex_);
    if (dropletBackDepthFbo_ != 0) glDeleteFramebuffers(1, &dropletBackDepthFbo_);
    if (dropletDepthTex_ != 0) glDeleteTextures(1, &dropletDepthTex_);
    if (dropletNormalTex_ != 0) glDeleteTextures(1, &dropletNormalTex_);
    if (dropletFbo_ != 0) glDeleteFramebuffers(1, &dropletFbo_);

    if (sceneDepthTex_ != 0) glDeleteTextures(1, &sceneDepthTex_);
    if (sceneColorTex_ != 0) glDeleteTextures(1, &sceneColorTex_);
    if (sceneFbo_ != 0) glDeleteFramebuffers(1, &sceneFbo_);

    dropletDepthTex_ = 0;
    dropletNormalTex_ = 0;
    dropletFbo_ = 0;
    dropletBackDepthTex_ = 0;
    dropletBackDepthFbo_ = 0;

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
    releaseEnvironmentMap();
    releaseRenderTargets();
    dropletCache_.clear();

    if (surfaceEbo_ != 0) glDeleteBuffers(1, &surfaceEbo_);
    if (surfaceVbo_ != 0) glDeleteBuffers(1, &surfaceVbo_);
    if (surfaceVao_ != 0) glDeleteVertexArrays(1, &surfaceVao_);
    if (fullscreenVao_ != 0) glDeleteVertexArrays(1, &fullscreenVao_);

    surfaceVao_ = 0;
    surfaceVbo_ = 0;
    surfaceEbo_ = 0;
    fullscreenVao_ = 0;
    surfaceIndexCount_ = 0;

    supportSurfaceShader_.reset();
    backgroundShader_.reset();
    dropletGBufferShader_.reset();
    compositeShader_.reset();
}

void RefractiveRenderer::renderEnvironmentBackground(const Camera& camera) {
    if (environmentTex_ == 0 || backgroundShader_.id() == 0 || fullscreenVao_ == 0) return;

    const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);
    backgroundShader_.use();
    backgroundShader_.setInt("uEnvironmentMap", 0);
    backgroundShader_.setMat4("uInvProj", glm::inverse(matrices.proj));
    backgroundShader_.setMat3("uInvViewRot", inverseViewRotation(matrices.view));

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, environmentTex_);

    glBindVertexArray(fullscreenVao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(0);
}

void RefractiveRenderer::renderSceneColorDepth(const Scene& scene, const Camera& camera) {
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
    glViewport(0, 0, std::max(width_, 1), std::max(height_, 1));
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);
    renderEnvironmentBackground(camera);

    if (scene.hasSurface()) {
        syncSurfaceMesh(scene.surface());
    }

    if (scene.hasSurface() && supportSurfaceShader_.id() != 0 &&
        surfaceVao_ != 0 && surfaceIndexCount_ > 0) {
        const SurfaceRenderParams& surfaceRender = scene.surface().renderParams();

        supportSurfaceShader_.use();
        supportSurfaceShader_.setInt("uEnvironmentMap", 0);
        supportSurfaceShader_.setMat4("uModel", matrices.model);
        supportSurfaceShader_.setMat4("uView", matrices.view);
        supportSurfaceShader_.setMat4("uProj", matrices.proj);
        supportSurfaceShader_.setVec3("uCameraPos", matrices.eye);
        supportSurfaceShader_.setFloat("uIor", static_cast<float>(surfaceRender.ior));
        supportSurfaceShader_.setFloat("uOpacity", static_cast<float>(surfaceRender.opacity));
        supportSurfaceShader_.setVec3("uTintColor", toGlm(surfaceRender.tintColor));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, environmentTex_);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(surfaceVao_);
        glDrawElements(GL_TRIANGLES, surfaceIndexCount_, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
        glDisable(GL_BLEND);
        glBindTexture(GL_TEXTURE_2D, 0);
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
    constexpr float depthClear = 1.0f;
    glClearBufferfv(GL_COLOR, 0, normalClear.data());
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

void RefractiveRenderer::renderDropletBackDepth(const Scene& scene, const Camera& camera) {
    glBindFramebuffer(GL_FRAMEBUFFER, dropletBackDepthFbo_);
    glViewport(0, 0, std::max(width_, 1), std::max(height_, 1));
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    constexpr float depthClear = 1.0f;
    glClearBufferfv(GL_DEPTH, 0, &depthClear);

    const GLboolean cullWasEnabled = glIsEnabled(GL_CULL_FACE);
    GLint previousCullFace = GL_BACK;
    glGetIntegerv(GL_CULL_FACE_MODE, &previousCullFace);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);

    if (dropletGBufferShader_.id() != 0) {
        dropletGBufferShader_.use();
        dropletGBufferShader_.setMat4("uView", matrices.view);
        dropletGBufferShader_.setMat4("uProj", matrices.proj);

        for (const auto& droplet : scene.droplets()) {
            dropletCache_.drawDroplet(droplet->id());
        }
    }

    glCullFace(static_cast<unsigned int>(previousCullFace));
    if (!cullWasEnabled) glDisable(GL_CULL_FACE);
    glUseProgram(0);
}

void RefractiveRenderer::compositeDroplets(const Scene&, const Camera& camera, const RenderParams& params) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width_, height_);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    if (compositeShader_.id() != 0 && fullscreenVao_ != 0) {
        const CameraMatrices matrices = buildCameraMatrices(camera, width_, height_);

        compositeShader_.use();
        compositeShader_.setInt("uSceneColor", 0);
        compositeShader_.setInt("uDropletNormal", 1);
        compositeShader_.setInt("uEnvironmentMap", 2);
        compositeShader_.setInt("uSceneDepth", 3);
        compositeShader_.setInt("uDropletDepth", 4);
        compositeShader_.setInt("uDropletBackDepth", 5);
        compositeShader_.setInt("uEnableThickness", params.enableThickness ? 1 : 0);
        compositeShader_.setInt("uDebugView", static_cast<int>(params.debugView));
        compositeShader_.setMat4("uInvProj", glm::inverse(matrices.proj));
        compositeShader_.setMat3("uInvViewRot", inverseViewRotation(matrices.view));
        compositeShader_.setFloat("uIor", static_cast<float>(params.ior));
        compositeShader_.setFloat("uNearPlane", kCameraNearPlane);
        compositeShader_.setFloat("uFarPlane", kCameraFarPlane);
        compositeShader_.setFloat("uRefractionScale", static_cast<float>(params.refractionScale));
        compositeShader_.setFloat("uFresnelBias", static_cast<float>(params.fresnelBias));
        compositeShader_.setFloat("uFresnelScale", static_cast<float>(params.fresnelScale));
        compositeShader_.setFloat("uFresnelPower", static_cast<float>(params.fresnelPower));
        compositeShader_.setFloat("uMaxThickness", static_cast<float>(params.maxThickness));
        compositeShader_.setFloat("uDebugDepthRange", static_cast<float>(params.debugDepthRange));
        compositeShader_.setFloat("uAbsorptionStrength", static_cast<float>(params.absorptionStrength));
        compositeShader_.setVec3("uAbsorptionColor", toGlm(params.absorptionColor));

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sceneColorTex_);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, dropletNormalTex_);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, environmentTex_);
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, sceneDepthTex_);
        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, dropletDepthTex_);
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_2D, dropletBackDepthTex_);

        glBindVertexArray(fullscreenVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);

        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, 0);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, 0);
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
