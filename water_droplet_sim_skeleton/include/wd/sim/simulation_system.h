#pragma once
#include "wd/core/scene.h"
#include "wd/sim/merge_split_controller.h"
#include "wd/sim/single_droplet_solver.h"

namespace wd {

class SimulationSystem {
public:
    SimulationSystem(SolverParams solverParams, MergeSplitController mergeSplit);

    void step(Scene& scene);

    double timeSec() const { return timeSec_; }
    const SimulationStats& stats() const { return stats_; }

    SingleDropletSolver& singleDropletSolver() { return solver_; }
    MergeSplitController& mergeSplitController() { return mergeSplit_; }

private:
    double timeSec_ = 0.0;
    SingleDropletSolver solver_;
    MergeSplitController mergeSplit_;
    SimulationStats stats_;
};

} // namespace wd
