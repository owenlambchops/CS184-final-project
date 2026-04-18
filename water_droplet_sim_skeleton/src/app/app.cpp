#include "wd/app/app.h"
#include "wd/sim/droplet_factory.h"
#include "wd/sim/droplet_template.h"
#include "wd/surface/plane_surface.h"

#include <iostream>

namespace wd {

int App::run() {
    initialize();

    // CPU-only smoke test. Replace this with the GLFW/OpenGL main loop.
    for (int i = 0; i < 10; ++i) {
        update();
        render();
    }

    std::cout << "Water droplet simulator skeleton initialized.\n";
    std::cout << "Droplet count: " << scene_.droplets().size() << "\n";
    if (sim_) {
        std::cout << "Last simulation step: " << sim_->stats().simMs << " ms\n";
        std::cout << "Mean volume error: " << sim_->stats().meanVolumeError << "\n";
    }
    return 0;
}

void App::initialize() {
    buildDefaultScene();
    renderer_ = std::make_unique<RefractiveRenderer>();
    renderer_->initialize(width_, height_);
    input_ = std::make_unique<InputRouter>(window_);
    dragInteractor_ = std::make_unique<DragInteractor>(dragField_);
    ui_ = std::make_unique<UiController>();
    logger_ = std::make_unique<ExperimentLogger>();
}

void App::buildDefaultScene() {
    scene_.setSurface(std::make_unique<PlaneSurface>(Vec3::Zero(), Vec3::UnitY()));

    auto composite = std::make_shared<CompositeForceField>();
    composite->addField(std::make_shared<ConstantForceField>(gravityLikeForce_));
    dragField_ = std::make_shared<DragForceField>();
    composite->addField(dragField_);
    scene_.setForceField(composite);

    auto tpl = DropletTemplate::CreateSphericalCap(8, 48, 0.25, 0.12);
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

void App::update() {
    if (input_) input_->beginFrame();
    if (dragInteractor_ && input_) {
        dragInteractor_->update(input_->state(), camera_, width_, height_, scene_);
    }
    if (sim_) sim_->step(scene_);
}

void App::render() {
    if (!renderer_) return;
    RenderStats stats = renderer_->render(scene_, camera_, renderParams_);
    if (logger_ && sim_) logger_->record(sim_->timeSec(), scene_, sim_->stats(), stats);
}

} // namespace wd
