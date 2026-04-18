#include "wd/sim/simulation_system.h"
#include <chrono>
#include <cmath>

namespace wd {

SimulationSystem::SimulationSystem(SolverParams solverParams, MergeSplitController mergeSplit)
    : solver_(solverParams), mergeSplit_(std::move(mergeSplit)) {}

void SimulationSystem::step(Scene& scene) {
    using Clock = std::chrono::high_resolution_clock;
    auto start = Clock::now();

    if (!scene.hasSurface() || !scene.hasForceField()) return;

    for (auto& d : scene.droplets()) {
        solver_.step(*d, scene.surface(), scene.forceField(), timeSec_);
    }

    mergeSplit_.process(scene.droplets(), scene.surface());

    VolumeCorrector volume;
    double sumErr = 0.0;
    double maxErr = 0.0;

    for (const auto& d : scene.droplets()) {
        double v = volume.computeClosedVolume(*d, scene.surface());
        double target = std::max(d->targetVolume(), 1e-12);
        double err = std::abs(v - target) / target;
        sumErr += err;
        maxErr = std::max(maxErr, err);
    }

    stats_.dropletCount = static_cast<int>(scene.droplets().size());
    stats_.meanVolumeError = scene.droplets().empty() ? 0.0 : sumErr / static_cast<double>(scene.droplets().size());
    stats_.maxVolumeError = maxErr;

    timeSec_ += solver_.params().dt;

    auto end = Clock::now();
    stats_.simMs = std::chrono::duration<double, std::milli>(end - start).count();
}

} // namespace wd
