#include "wd/sim/single_droplet_solver.h"
#include <algorithm>
#include <cmath>

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
        if (params_.enableCollision) collision_.apply(drop, surface, params_.collisionPushoutEps, params_.adhesionDistance, dt);


        // Volume correction must run on the live droplet state.
        // This preserves local-first/global-second logic in VolumeCorrector::apply:
        // local velocity correction uses current U, then global position correction updates X.
        if (params_.enableVolumeCorrect) {
            drop.updateDerived();
            volume_.apply(drop, surface, dt);
        }
        // TODO(remesh-step2): Insert topology-changing remesh pass here (split/collapse/flip).
        // Suggested order:
        // 1) split long edges
        // 2) collapse short edges
        // 3) optional edge flips for valence quality
        // 4) rebuild edges + remove degenerate faces + updateDerived()

        drop.updateDerived();
    }

    drop.updateDerived();
}

MatX3d SingleDropletSolver::computeAcceleration(
        const Droplet& drop,
        const ISurface& surface,
        const IForceField& field,
        double timeSec) const {
    Droplet internalDrop = drop;
    internalDrop.velocities() = MatX3d::Zero(internalDrop.positions().rows(), 3);

    if (params_.enableCurvatureFlow) curvature_.apply(internalDrop, surface, 1.0);
    if (params_.enableContactAngle) contact_.apply(internalDrop, surface, 1.0, params_.adhesionDistance);

    MatX3d aInternal = internalDrop.velocities();
    const double maxA = std::max(params_.maxInternalAccel, 0.0);
    if (maxA > 0.0) {
        for (int i = 0; i < aInternal.rows(); ++i) {
            Vec3 ai = aInternal.row(i).transpose();
            const double n = ai.norm();
            if (!std::isfinite(n)) {
                aInternal.row(i).setZero();
                continue;
            }
            if (n > maxA) {
                ai *= (maxA / std::max(n, 1e-8));
                aInternal.row(i) = ai.transpose();
            }
        }
    }

    MatX3d aExternal = MatX3d::Zero(drop.positions().rows(), 3);
    if (params_.enableExternalForce) {
        Droplet externalDrop = drop;
        externalDrop.velocities() = MatX3d::Zero(externalDrop.positions().rows(), 3);
        external_.apply(externalDrop, surface, field, timeSec, 1.0);
        aExternal = externalDrop.velocities();
    }

    MatX3d aTotal = aInternal + aExternal;
    for (int i = 0; i < aTotal.rows(); ++i) {
        const Vec3 ai = aTotal.row(i).transpose();
        if (!std::isfinite(ai.x()) || !std::isfinite(ai.y()) || !std::isfinite(ai.z())) {
            aTotal.row(i).setZero();
        }
    }
    return aTotal;
}

} // namespace wd
