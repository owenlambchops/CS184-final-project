#include "wd/app/app.h"
#include "wd/sim/droplet_factory.h"
#include "wd/sim/droplet_template.h"
#include "wd/surface/plane_surface.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace wd {

namespace {

constexpr double kMaxCameraDt = 0.1;
constexpr double kCameraMoveSpeed = 2.0;
constexpr double kCameraScrollDistance = 0.12;
constexpr double kCameraScrollSmoothingTime = 0.18;
constexpr double kCameraMouseSensitivity = 0.0025;

} // namespace

int App::run() {
    if (!initializeWindow()) return 1;
    if (!initialize()) {
        shutdown();
        return 1;
    }

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        processInput();
        update();
        render();
        glfwSwapBuffers(window_);
    }

    shutdown();
    return 0;
}

bool App::initializeWindow() {
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialize GLFW.\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    window_ = glfwCreateWindow(width_, height_, "Water Droplet Simulator", nullptr, nullptr);
    if (window_ == nullptr) {
        std::cerr << "Failed to create GLFW window.\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) {
        std::cerr << "Failed to load OpenGL functions via glad.\n";
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &App::framebufferSizeCallback);
    glfwSetScrollCallback(window_, &App::scrollCallback);
    glfwGetFramebufferSize(window_, &width_, &height_);

    initializeGlState();
    glViewport(0, 0, width_, height_);
    return true;
}

bool App::initialize() {
    buildDefaultScene();
    camera_.setPosition(Vec3(2.2, 1.35, 2.2));
    camera_.lookAt(Vec3::Zero());
    lastFrameTimeSec_ = glfwGetTime();

    renderer_ = std::make_unique<RefractiveRenderer>();
    if (!renderer_->initialize(width_, height_)) {
        std::cerr << "Failed to initialize renderer.\n";
        return false;
    }

    input_ = std::make_unique<InputRouter>(window_);
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    ui_ = std::make_unique<UiController>();
    logger_ = std::make_unique<ExperimentLogger>();

    return true;
}

void App::initializeGlState() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);
}

void App::buildDefaultScene() {
    scene_.setSurface(std::make_unique<PlaneSurface>(Vec3::Zero(), Vec3::UnitY()));

    auto composite = std::make_shared<CompositeForceField>();
    composite->addField(std::make_shared<ConstantForceField>(gravityLikeForce_));
    dragField_ = std::make_shared<DragForceField>();
    composite->addField(dragField_);
    scene_.setForceField(composite);

    auto tpl = DropletTemplate::CreateSphericalCap(8, 48, 0.22, 0.20);
    DropletFactory factory(tpl);

    SpawnDesc desc;
    desc.anchorWorld = Vec3(0.0, 0.0, 0.0);
    desc.initialVelocity = Vec3::Zero();
    desc.targetVolume = 0.01;
    desc.material = defaultMaterial_;
    scene_.droplets().push_back(factory.spawn(1, desc, scene_.surface()));

    MergeSplitController mergeSplit(factory);
    sim_ = std::make_unique<SimulationSystem>(solverParams_, std::move(mergeSplit));
}

void App::restartSimulation() {
    scene_ = Scene{};
    sim_.reset();
    dragField_.reset();

    buildDefaultScene();
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    singleStepRequested_ = false;
}

void App::processInput() {
    if (window_ == nullptr) return;

    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    const bool pauseKeyDown = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (pauseKeyDown && !pauseKeyWasDown_) {
        paused_ = !paused_;
    }
    pauseKeyWasDown_ = pauseKeyDown;

    const bool stepKeyDown = glfwGetKey(window_, GLFW_KEY_N) == GLFW_PRESS;
    if (paused_ && stepKeyDown && !stepKeyWasDown_) {
        singleStepRequested_ = true;
    }
    stepKeyWasDown_ = stepKeyDown;

    const bool restartKeyDown = glfwGetKey(window_, GLFW_KEY_R) == GLFW_PRESS;
    if (restartKeyDown && !restartKeyWasDown_) {
        restartSimulation();
    }
    restartKeyWasDown_ = restartKeyDown;

    if (input_) {
        double mouseX = 0.0;
        double mouseY = 0.0;
        glfwGetCursorPos(window_, &mouseX, &mouseY);
        input_->setMousePosition(mouseX, mouseY);
        input_->setLeftButton(glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        input_->setRightButton(glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    }
}

void App::updateCameraControls(double dt) {
    if (!input_ || window_ == nullptr) return;

    const InputState& state = input_->state();

    if (state.rightDown) {
        if (!cameraRightDragActive_ || state.rightPressed) {
            cameraRightDragActive_ = true;
        } else {
            const double dx = state.mouseX - lastCameraMouseX_;
            const double dy = state.mouseY - lastCameraMouseY_;
            camera_.rotate(dx * kCameraMouseSensitivity, -dy * kCameraMouseSensitivity);
        }

        lastCameraMouseX_ = state.mouseX;
        lastCameraMouseY_ = state.mouseY;
    } else {
        cameraRightDragActive_ = false;
    }

    Vec3 move = Vec3::Zero();
    if (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS) move += camera_.forward();
    if (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS) move -= camera_.forward();
    if (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS) move += camera_.right();
    if (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS) move -= camera_.right();

    if (move.norm() > 1e-8) {
        camera_.move(move.normalized() * kCameraMoveSpeed * dt);
    }

    if (state.scrollY != 0.0) {
        cameraScrollVelocity_ += state.scrollY * kCameraScrollDistance / kCameraScrollSmoothingTime;
    }

    if (std::abs(cameraScrollVelocity_) > 1e-5) {
        camera_.move(camera_.forward() * cameraScrollVelocity_ * dt);
        cameraScrollVelocity_ *= std::exp(-dt / kCameraScrollSmoothingTime);
    } else {
        cameraScrollVelocity_ = 0.0;
    }
}

void App::update() {
    const double now = glfwGetTime();
    const double dt = std::clamp(now - lastFrameTimeSec_, 0.0, kMaxCameraDt);
    lastFrameTimeSec_ = now;

    if (input_) input_->beginFrame();
    updateCameraControls(dt);
    if (dragInteractor_ && input_) {
        dragInteractor_->update(input_->state(), camera_, width_, height_, scene_);
    }
    if (sim_ && (!paused_ || singleStepRequested_)) {
        sim_->step(scene_);
        singleStepRequested_ = false;
    }
    if (input_) input_->clearFrameDeltas();
}

void App::render() {
    RenderStats stats{};
    stats.frameWidth = width_;
    stats.frameHeight = height_;

    if (renderer_) {
        stats = renderer_->render(scene_, camera_, renderParams_);
    }

    if (logger_ && sim_) logger_->record(sim_->timeSec(), scene_, sim_->stats(), stats);
}

void App::shutdown() {
    logger_.reset();
    ui_.reset();
    dragInteractor_.reset();
    input_.reset();
    sim_.reset();
    renderer_.reset();

    if (window_ != nullptr) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    glfwTerminate();
}

void App::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(window));
    if (app == nullptr) return;

    app->width_ = width;
    app->height_ = height;
    glViewport(0, 0, width, height);
    if (app->renderer_) app->renderer_->resize(width, height);
}

void App::scrollCallback(GLFWwindow* window, double, double yoffset) {
    auto* app = static_cast<App*>(glfwGetWindowUserPointer(window));
    if (app == nullptr || !app->input_) return;
    app->input_->addScroll(yoffset);
}

} // namespace wd
