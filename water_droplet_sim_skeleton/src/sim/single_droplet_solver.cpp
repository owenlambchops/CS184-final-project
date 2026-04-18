#include "wd/sim/single_droplet_solver.h"
#include <algorithm>

namespace wd {

SingleDropletSolver::SingleDropletSolver(SolverParams params) : params_(params) {}

void SingleDropletSolver::step(Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec) {
    int substeps = std::max(1, params_.substeps);
    double dt = params_.dt / static_cast<double>(substeps);

    for (int i = 0; i < substeps; ++i) {
        if (params_.enableExternalForce) external_.apply(drop, surface, field, timeSec, dt);
        semiImplicitIntegrate(drop, dt);
        if (params_.enableCollision) collision_.apply(drop, surface, params_.collisionPushoutEps);
        if (params_.enableViscosity) viscosity_.apply(drop, dt);
        if (params_.enableCurvatureFlow) curvature_.apply(drop, dt);
        if (params_.enableContactAngle) contact_.apply(drop, surface, dt);
        if (params_.enableVolumeCorrect) volume_.apply(drop, surface, dt);
        if (params_.enableCollision) collision_.apply(drop, surface, params_.collisionPushoutEps);
    }

    VolumeCorrector vc;
    drop.derived().currentVolume = vc.computeClosedVolume(drop, surface);
    drop.updateDerived();
}

void SingleDropletSolver::semiImplicitIntegrate(Droplet& drop, double dt) const {
    drop.positions() += dt * drop.velocities();
}

} // namespace wd
