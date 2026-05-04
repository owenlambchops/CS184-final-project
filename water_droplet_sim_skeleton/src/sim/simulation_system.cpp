#include "wd/sim/simulation_system.h"
#include <algorithm>
#include "wd/forces/droplet_drag_force_field.h"
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
        if (auto* ddf = scene.compositeForceField().findField<DropletDragForceField>()) {
            ddf->setActiveDropletId(d->id());
        }
        solver_.step(*d, scene.surface(), scene.forceField(), timeSec_);
    }

    mergeSplit_.process(scene.droplets(), scene.surface());

    auto& droplets = scene.droplets();
    droplets.erase(
        std::remove_if(
            droplets.begin(),
            droplets.end(),
            [](const std::unique_ptr<Droplet>& d) {
                return d && d->derived().centerOfMass.y() < -5.0;
            }),
        droplets.end());

    // VolumeCorrector volume;
    double sumErr = 0.0;
    double maxErr = 0.0;

    for (const auto& d : droplets) {
        // double v = volume.computeClosedVolume(*d, scene.surface());
        const double v = d->derived().currentVolume;
        double target = std::max(d->targetVolume(), 1e-12);
        double err = std::abs(v - target) / target;
        sumErr += err;
        maxErr = std::max(maxErr, err);
    }

    stats_.dropletCount = static_cast<int>(droplets.size());
    stats_.meanVolumeError = droplets.empty() ? 0.0 : sumErr / static_cast<double>(droplets.size());
    stats_.maxVolumeError = maxErr;

    timeSec_ += solver_.params().dt;

    auto end = Clock::now();
    stats_.simMs = std::chrono::duration<double, std::milli>(end - start).count();
}



} // namespace wd
