#pragma once
#include "wd/core/scene.h"
#include "wd/experiments/experiment_logger.h"
#include "wd/interaction/drag_interactor.h"
#include "wd/interaction/input_router.h"
#include "wd/render/droplet_gpu_cache.h"
#include "wd/render/refractive_renderer.h"
#include "wd/sim/simulation_system.h"
#include "wd/ui/ui_controller.h"

struct GLFWwindow;

namespace wd {

class App {
public:
    int run();

private:
    bool initializeWindow();
    bool initialize();
    void initializeGlState();
    bool initializeRenderResources();
    void initializePlaneMesh();
    void buildDefaultScene();
    void restartSimulation();
    void processInput();
    void update();
    void render();
    void updateHudTitle();
    void destroyRenderResources();
    void shutdown();

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

    GLFWwindow* window_ = nullptr;
    int width_ = 1280;
    int height_ = 720;

    Scene scene_;
    Camera camera_;

    SolverParams solverParams_;
    RenderParams renderParams_;
    MaterialParams defaultMaterial_;
    Vec3 gravityLikeForce_ = Vec3(0.0, -9.81, 0.0);
    // Vec3 gravityLikeForce_ = Vec3(-7.0, -3.0, 0.0);
    // Vec3 gravityLikeForce_ = Vec3(0.0, 0.0, 0.0);

    std::shared_ptr<DragForceField> dragField_;

    DropletGpuCache dropletCache_;
    std::unique_ptr<InputRouter> input_;
    std::unique_ptr<DragInteractor> dragInteractor_;
    std::unique_ptr<UiController> ui_;
    std::unique_ptr<SimulationSystem> sim_;
    std::unique_ptr<RefractiveRenderer> renderer_;
    std::unique_ptr<ExperimentLogger> logger_;

    unsigned int dropletProgram_ = 0;
    unsigned int planeProgram_ = 0;
    unsigned int planeVao_ = 0;
    unsigned int planeVbo_ = 0;
    unsigned int planeEbo_ = 0;
    int planeIndexCount_ = 0;

    bool paused_ = false;
    bool pauseKeyWasDown_ = false;
    bool stepKeyWasDown_ = false;
    bool restartKeyWasDown_ = false;
    bool singleStepRequested_ = false;
    bool wireframe_ = false;
    bool wireframeKeyWasDown_ = false;
    bool incKeyWasDown_ = false;
    bool decKeyWasDown_ = false;
    bool tuneSelectUpWasDown_ = false;
    bool tuneSelectDownWasDown_ = false;
    int tuneIndex_ = 0;
};

} // namespace wd
