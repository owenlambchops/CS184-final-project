#pragma once
#include "wd/core/scene.h"
#include "wd/experiments/experiment_logger.h"
#include "wd/interaction/drag_interactor.h"
#include "wd/interaction/input_router.h"
#include "wd/render/refractive_renderer.h"
#include "wd/sim/simulation_system.h"
#include "wd/ui/ui_controller.h"

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
    void initializeGlState();
    void buildDefaultScene();
    void spawnDropletAt(const Vec3& anchorWorld);
    void restartSimulation();
    void processInput();
    void updateCameraControls(double dt);
    void update();
    void render();
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
    Vec3 gravityLikeForce_ = Vec3(0.0, 0.0, 0.0);

    std::shared_ptr<ConstantForceField> gravityField_;
    std::shared_ptr<DragForceField> dragField_;
    std::shared_ptr<const DropletTemplate> dropletTemplate_;

    std::unique_ptr<InputRouter> input_;
    std::unique_ptr<DragInteractor> dragInteractor_;
    std::unique_ptr<UiController> ui_;
    std::unique_ptr<SimulationSystem> sim_;
    std::unique_ptr<RefractiveRenderer> renderer_;
    std::unique_ptr<ExperimentLogger> logger_;

    bool paused_ = false;
    bool pauseKeyWasDown_ = false;
    bool stepKeyWasDown_ = false;
    bool restartKeyWasDown_ = false;
    bool singleStepRequested_ = false;
    bool imguiInitialized_ = false;
    bool cameraRightDragActive_ = false;
    int nextDropletId_ = 1;
    double lastCameraMouseX_ = 0.0;
    double lastCameraMouseY_ = 0.0;
    double lastFrameTimeSec_ = 0.0;
    double cameraScrollVelocity_ = 0.0;
};

} // namespace wd
