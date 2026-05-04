#include "wd/app/app.h"
#include "wd/experiments/screenshot_writer.h"
#include "wd/sim/droplet_factory.h"
#include "wd/sim/droplet_template.h"
#include "wd/surface/plane_surface.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace wd {

namespace {

constexpr double kMaxCameraDt = 0.1;
constexpr double kCameraMoveSpeed = 2.0;
constexpr double kCameraScrollDistance = 0.12;
constexpr double kCameraScrollSmoothingTime = 0.18;
constexpr double kCameraMouseSensitivity = 0.0025;
constexpr const char* kExperimentOutputDir = "experiments";

std::string makeTimestampedRunName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &nowTime);
#else
    localtime_r(&nowTime, &localTime);
#endif

    std::ostringstream name;
    name << "run_" << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return name.str();
}

bool ensureExperimentOutputDir() {
    std::error_code error;
    std::filesystem::create_directories(kExperimentOutputDir, error);
    if (error) {
        std::cerr << "Failed to create experiment output directory: "
                  << error.message() << "\n";
        return false;
    }
    return true;
}

std::filesystem::path experimentOutputPath(const std::string& filename) {
    return std::filesystem::path(kExperimentOutputDir) / filename;
}

std::string makeScreenshotFilename(const std::string& runName, int index) {
    std::ostringstream filename;
    filename << runName << "_screenshot_" << std::setw(4) << std::setfill('0') << index << ".png";
    return filename.str();
}

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

    if (!initializeImGui()) return false;

    input_ = std::make_unique<InputRouter>(window_);
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    planeTiltInteractor_ = std::make_unique<PlaneTiltInteractor>();
    ui_ = std::make_unique<UiController>();
    logger_ = std::make_unique<ExperimentLogger>();
    runName_ = makeTimestampedRunName();
    logger_->beginRun(runName_);

    return true;
}

bool App::initializeImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsLight();
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(1.0f, 1.0f, 1.0f, 0.98f);
    colors[ImGuiCol_PopupBg] = ImVec4(1.0f, 1.0f, 1.0f, 0.98f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.97f, 0.97f, 0.97f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.93f, 0.93f, 0.93f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.95f, 0.95f, 0.95f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.90f, 0.90f, 0.90f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.94f, 0.94f, 0.94f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.89f, 0.89f, 0.89f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.84f, 0.84f, 0.84f, 1.0f);
    colors[ImGuiCol_Text] = ImVec4(0.08f, 0.08f, 0.08f, 1.0f);
    colors[ImGuiCol_Border] = ImVec4(0.82f, 0.82f, 0.82f, 1.0f);

    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend.\n";
        ImGui::DestroyContext();
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        std::cerr << "Failed to initialize ImGui OpenGL backend.\n";
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        return false;
    }

    imguiInitialized_ = true;
    return true;
}

void App::initializeGlState() {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.08f, 0.10f, 0.12f, 1.0f);
}

void App::buildDefaultScene() {
    scene_.setSurface(std::make_unique<PlaneSurface>(Vec3::Zero(), Vec3::UnitY()));
    scene_.surface().material().friction = 0.02;

    auto composite = std::make_shared<CompositeForceField>();
    gravityField_ = std::make_shared<ConstantForceField>(gravityLikeForce_);
    composite->addField(gravityField_);
    dragField_ = std::make_shared<DragForceField>();
    composite->addField(dragField_);
    scene_.setForceField(composite);

    dropletTemplate_ = DropletTemplate::CreateSphericalMesh(3, 0.20);
    DropletFactory factory(dropletTemplate_);
    MergeSplitController mergeSplit(factory);
    sim_ = std::make_unique<SimulationSystem>(solverParams_, std::move(mergeSplit));
}

void App::spawnDropletAt(const Vec3& anchorWorld) {
    if (!scene_.hasSurface() || !dropletTemplate_) return;

    DropletFactory factory(dropletTemplate_);
    SpawnDesc desc;
    desc.anchorWorld = anchorWorld;
    desc.initialVelocity = Vec3::Zero();
    desc.targetVolume = 0.0;
    desc.material = defaultMaterial_;

    scene_.droplets().push_back(factory.spawn(nextDropletId_++, desc, scene_.surface()));
}

void App::restartSimulation() {
    scene_ = Scene{};
    sim_.reset();
    gravityField_.reset();
    dragField_.reset();
    dropletTemplate_.reset();
    nextDropletId_ = 1;

    buildDefaultScene();
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    planeTiltInteractor_ = std::make_unique<PlaneTiltInteractor>();
    interactionsEnabled_ = true;
    singleStepRequested_ = false;
}

void App::processInput() {
    if (window_ == nullptr) return;

    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    const bool imguiWantsKeyboard = imguiInitialized_ && ImGui::GetIO().WantCaptureKeyboard;
    if (!imguiWantsKeyboard) {
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

    } else {
        pauseKeyWasDown_ = false;
        stepKeyWasDown_ = false;
        restartKeyWasDown_ = false;
    }

    const bool screenshotKeyDown = glfwGetKey(window_, GLFW_KEY_G) == GLFW_PRESS;
    if (screenshotKeyDown && !screenshotKeyWasDown_) {
        screenshotRequested_ = true;
    }
    screenshotKeyWasDown_ = screenshotKeyDown;

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
    simulationAdvancedThisFrame_ = false;

    const double now = glfwGetTime();
    const double dt = std::clamp(now - lastFrameTimeSec_, 0.0, kMaxCameraDt);
    lastFrameTimeSec_ = now;

    if (input_) input_->beginFrame();

    // Always update mouse position for interaction
    double mouseX = 0.0, mouseY = 0.0;
    if (window_) {
        glfwGetCursorPos(window_, &mouseX, &mouseY);
        input_->setMousePosition(mouseX, mouseY);
    }
    const bool imguiWantsMouse = imguiInitialized_ && ImGui::GetIO().WantCaptureMouse;
    if (!imguiWantsMouse) {
        updateCameraControls(dt);
    } else {
        cameraRightDragActive_ = false;
    }

    if (interactionsEnabled_ && !imguiWantsMouse && dragInteractor_ && input_) {
        dragInteractor_->update(input_->state(), camera_, width_, height_, scene_);
    }
    if (interactionsEnabled_ && !imguiWantsMouse && planeTiltInteractor_ && input_) {
        planeTiltInteractor_->update(input_->state(), width_, height_, dt, scene_, planeTiltParams_);
    }

    if (sim_ && (!paused_ || singleStepRequested_)) {
        sim_->step(scene_);
        simulationAdvancedThisFrame_ = true;
        singleStepRequested_ = false;
    }
    if (input_) input_->clearFrameDeltas();
}

void App::render() {
    RenderStats stats{};
    stats.frameWidth = width_;
    stats.frameHeight = height_;

    bool drawUi = false;
    if (imguiInitialized_ && ui_ && sim_ && scene_.hasSurface()) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        auto* planeSurface = dynamic_cast<PlaneSurface*>(&scene_.surface());
        const double planeSideLength = planeSurface ? planeSurface->sideLength() : 0.0;
        UiActions actions = ui_->draw(
            solverParams_,
            renderParams_,
            defaultMaterial_,
            scene_.surface().material(),
            scene_.surface().renderParams(),
            planeSideLength,
            planeTiltParams_.enabled,
            planeTiltParams_.maxTiltDeg,
            planeTiltParams_.responsiveness,
            planeTiltParams_.axisScaleX,
            planeTiltParams_.axisScaleZ,
            gravityLikeForce_,
            sim_->mergeSplitController());
        sim_->singleDropletSolver().params() = solverParams_;
        if (actions.setPlaneSideLength && planeSurface) {
            planeSurface->setSideLength(actions.planeSideLength);
        }
        // Plane tilt always enabled
        planeTiltParams_.enabled = true;
        if (actions.setPlaneTiltMaxDeg) planeTiltParams_.maxTiltDeg = actions.planeTiltMaxDeg;
        if (actions.setPlaneTiltResponsiveness) planeTiltParams_.responsiveness = actions.planeTiltResponsiveness;
        if (actions.setPlaneTiltAxisScaleX) planeTiltParams_.axisScaleX = actions.planeTiltAxisScaleX;
        if (actions.setPlaneTiltAxisScaleZ) planeTiltParams_.axisScaleZ = actions.planeTiltAxisScaleZ;
        if (actions.resetPlaneAndDisableInteraction) {
            restartSimulation();
            // Plane tilt and interaction always enabled
            interactionsEnabled_ = true;
            if (dragField_) dragField_->setActive(true);
        }
        if (actions.disable_tilt) {
            planeTiltParams_.enabled = false;
        }
        if (actions.createDroplet) spawnDropletAt(actions.spawnAnchor);
        if (actions.applyGravity) {
            gravityLikeForce_ = actions.gravityForce;
            if (gravityField_) gravityField_->setForce(gravityLikeForce_);
        }
        if (actions.saveScreenshot) screenshotRequested_ = true;

        ImGui::Render();
        drawUi = true;
    }

    if (renderer_) {
        stats = renderer_->render(scene_, camera_, renderParams_);
    }

    if (drawUi) {
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    if (screenshotRequested_) saveScreenshot();

    if (logger_ && sim_ && simulationAdvancedThisFrame_) {
        logger_->record(sim_->timeSec(), scene_, sim_->stats(), stats);
    }
}

void App::saveExperimentCsv() const {
    if (!logger_ || runName_.empty() || !ensureExperimentOutputDir()) return;

    const auto path = experimentOutputPath(runName_ + ".csv");
    if (!logger_->saveCsv(path.string())) {
        std::cerr << "Failed to save experiment CSV: " << path << "\n";
    }
}

void App::saveScreenshot() {
    screenshotRequested_ = false;
    if (runName_.empty() || !ensureExperimentOutputDir()) return;

    const auto path = experimentOutputPath(makeScreenshotFilename(runName_, screenshotCounter_));
    if (!saveFramebufferPng(path.string(), width_, height_)) {
        std::cerr << "Failed to save screenshot: " << path << "\n";
        return;
    }

    std::cout << "Saved screenshot: " << path << "\n";
    ++screenshotCounter_;
}

void App::shutdownImGui() {
    if (!imguiInitialized_) return;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    imguiInitialized_ = false;
}

void App::shutdown() {
    saveExperimentCsv();

    logger_.reset();
    ui_.reset();
    dragInteractor_.reset();
    planeTiltInteractor_.reset();
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
