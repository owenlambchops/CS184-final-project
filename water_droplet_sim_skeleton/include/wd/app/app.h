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

class App {
public:
    int run();

private:
    void initialize();
    void buildDefaultScene();
    void update();
    void render();

    GLFWwindow* window_ = nullptr;
    int width_ = 1280;
    int height_ = 720;

    Scene scene_;
    Camera camera_;

    SolverParams solverParams_;
    RenderParams renderParams_;
    MaterialParams defaultMaterial_;
    Vec3 gravityLikeForce_ = Vec3(0.0, -9.8, 0.0);

    std::shared_ptr<DragForceField> dragField_;

    std::unique_ptr<InputRouter> input_;
    std::unique_ptr<DragInteractor> dragInteractor_;
    std::unique_ptr<UiController> ui_;
    std::unique_ptr<SimulationSystem> sim_;
    std::unique_ptr<RefractiveRenderer> renderer_;
    std::unique_ptr<ExperimentLogger> logger_;
};

} // namespace wd
