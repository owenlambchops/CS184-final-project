#include "wd/sim/single_droplet_solver.h"
#include <algorithm>

namespace wd {

SingleDropletSolver::SingleDropletSolver(SolverParams params) : params_(params) {}

void SingleDropletSolver::step(Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec) {
    // Python parity path: explicit Euler update order; Verlet remains intentionally unimplemented here.
    int substeps = std::max(1, params_.substeps);
    double dt = params_.dt / static_cast<double>(substeps);

    for (int i = 0; i < substeps; ++i) {
        drop.updateDerived();
        if (params_.enableViscosity) viscosity_.apply(drop, dt);
        if (params_.enableCurvatureFlow) curvature_.apply(drop, surface, dt);
        if (params_.enableContactAngle) contact_.apply(drop, surface, dt, params_.adhesionDistance);
        if (params_.enableExternalForce) external_.apply(drop, surface, field, timeSec, dt);

        double speedClamp = std::max(params_.maxVelocity, 0.0);
        if (speedClamp > 0.0) {
            auto& U = drop.velocities();
            U = U.array().min(speedClamp).max(-speedClamp);
        }

        semiImplicitIntegrate(drop, dt);
        if (params_.enableCollision) {
            collision_.apply(
                drop, surface, params_.collisionPushoutEps, params_.adhesionDistance, dt);
        }
        if (params_.enableVolumeCorrect) volume_.apply(drop, surface, dt);
    }

    VolumeCorrector vc;
    drop.derived().currentVolume = vc.computeClosedVolume(drop, surface);
    drop.updateDerived();
}

void SingleDropletSolver::semiImplicitIntegrate(Droplet& drop, double dt) const {
    drop.positions() += dt * drop.velocities();
}

} // namespace wd
