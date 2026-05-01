#include "wd/sim/single_droplet_solver.h"
#include <algorithm>

namespace wd {

SingleDropletSolver::SingleDropletSolver(SolverParams params) 
    : params_(params) {}

void SingleDropletSolver::step(Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec) {
    int substeps = std::max(1, params_.substeps);
    double dt = params_.dt / static_cast<double>(substeps);

    for (int i = 0; i < substeps; ++i) {
        const double substepTime = timeSec + static_cast<double>(i) * dt;
        drop.updateDerived();

        MatX3d a0 = computeAcceleration(drop, surface, field, substepTime);
        auto& X = drop.positions();
        auto& U = drop.velocities();
        const MatX3d X0 = X;
        const MatX3d U0 = U;

        X = X0 + U0 * dt + a0 * (0.5 * dt * dt);
        drop.updateDerived();

        MatX3d a1 = computeAcceleration(drop, surface, field, substepTime + dt);
        U = U0 + (a0 + a1) * (0.5 * dt);

        double speedClamp = std::max(params_.maxVelocity, 0.0);
        if (speedClamp > 0.0) {
            U = U.array().min(speedClamp).max(-speedClamp);
        }

        if (params_.enableViscosity) viscosity_.apply(drop, dt);

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

MatX3d SingleDropletSolver::computeAcceleration(
        const Droplet& drop,
        const ISurface& surface,
        const IForceField& field,
        double timeSec) const {
    Droplet tmp = drop;
    tmp.velocities() = MatX3d::Zero(tmp.positions().rows(), 3);

    if (params_.enableCurvatureFlow) curvature_.apply(tmp, surface, 1.0);
    if (params_.enableContactAngle) contact_.apply(tmp, surface, 1.0, params_.adhesionDistance);
    if (params_.enableExternalForce) external_.apply(tmp, surface, field, timeSec, 1.0);
    if (params_.enableVolumeCorrect) volume_.apply(tmp, surface, 1.0);

    return tmp.velocities();
}

} // namespace wd
