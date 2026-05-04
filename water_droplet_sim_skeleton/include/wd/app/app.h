#pragma once

#include "wd/core/scene.h"
#include "wd/experiments/experiment_logger.h"
#include "wd/forces/droplet_drag_force_field.h"
#include "wd/interaction/drag_interactor.h"
#include "wd/interaction/droplet_drag_interactor.h"
#include "wd/interaction/input_router.h"
#include "wd/interaction/plane_tilt_interactor.h"
#include "wd/render/droplet_gpu_cache.h"
#include "wd/render/refractive_renderer.h"
#include "wd/sim/simulation_system.h"
#include "wd/ui/ui_controller.h"

#include <memory>
#include <string>

struct GLFWwindow;

namespace wd {

class ConstantForceField;
class DropletTemplate;

class App {
public:
    int run();

private:
    bool initializeWindow();
    bool initialize();
    bool initializeImGui();
    bool initializeRenderResources();
    void initializeGlState();
    void initializePlaneMesh();
    void buildDefaultScene();
    void spawnDropletAt(const Vec3& anchorWorld);
    void restartSimulation();
    void processInput();
    void updateCameraControls(double dt);
    void update();
    void render();
    void rebuildPlaneGpuMesh();
    void destroyRenderResources();
    void shutdownImGui();
    void shutdown();

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    GLFWwindow* window_ = nullptr;
    int width_ = 1280;
    int height_ = 720;

    Scene scene_;
    Camera camera_;

    SolverParams solverParams_;
    RenderParams renderParams_;
    MaterialParams defaultMaterial_;
    Vec3 gravityLikeForce_ = Vec3(0.0, -9.81, 0.0);

    std::shared_ptr<ConstantForceField> gravityField_;
    std::shared_ptr<DragForceField> dragField_;
    std::shared_ptr<DropletDragForceField> dropletDragField_;
    std::shared_ptr<const DropletTemplate> dropletTemplate_;
    int nextDropletId_ = 1;

    std::unique_ptr<InputRouter> input_;
    std::unique_ptr<DragInteractor> dragInteractor_;
    std::unique_ptr<DropletDragInteractor> dropletDragInteractor_;
    std::unique_ptr<PlaneTiltInteractor> planeTiltInteractor_;
    std::unique_ptr<UiController> ui_;
    std::unique_ptr<SimulationSystem> sim_;
    std::unique_ptr<RefractiveRenderer> renderer_;
    std::unique_ptr<ExperimentLogger> logger_;

    bool imguiInitialized_ = false;
    std::string runName_;

    bool paused_ = false;
    bool pauseKeyWasDown_ = false;
    bool stepKeyWasDown_ = false;
    bool restartKeyWasDown_ = false;
    bool screenshotKeyWasDown_ = false;
    bool singleStepRequested_ = false;
    bool screenshotRequested_ = false;

    bool cameraRightDragActive_ = false;
    double lastCameraMouseX_ = 0.0;
    double lastCameraMouseY_ = 0.0;
    double cameraScrollVelocity_ = 0.0;

    double lastFrameTimeSec_ = 0.0;
    bool simulationAdvancedThisFrame_ = false;

    unsigned int dropletProgram_ = 0;
    unsigned int planeProgram_ = 0;
    unsigned int planeVao_ = 0;
    unsigned int planeVbo_ = 0;
    unsigned int planeEbo_ = 0;
    int planeIndexCount_ = 0;
    DropletGpuCache dropletCache_;
};

} // namespace wd
