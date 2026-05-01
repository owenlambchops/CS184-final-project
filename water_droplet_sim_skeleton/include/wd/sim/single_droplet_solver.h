#pragma once
#include "wd/sim/operators.h"
#include <unordered_map>

namespace wd {

class SingleDropletSolver {
public:
    explicit SingleDropletSolver(SolverParams params = {});

    void step(Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec);

    const SolverParams& params() const { return params_; }
    SolverParams& params() { return params_; }

private:
    void semiImplicitIntegrate(Droplet& drop, double dt) const;
    MatX3d computeAcceleration(const Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec) const;

    SolverParams params_;
    ExternalForceOperator external_;
    CollisionProjector collision_;
    ViscosityOperator viscosity_;
    CurvatureFlowOperator curvature_;
    ContactLineOperator contact_;
    VolumeCorrector volume_;
    std::unordered_map<int, MatX3d> accelCache_;
};

} // namespace wd
