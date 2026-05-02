#pragma once
#include "wd/forces/iforce_field.h"
#include "wd/sim/droplet.h"
#include "wd/surface/isurface.h"

namespace wd {

class ExternalForceOperator {
public:
    void apply(Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec, double dt) const;
};

class CollisionProjector {
public:
    void apply(Droplet& drop, const ISurface& surface, double pushoutEps, double adhesionDist, double dt) const;
};

class ViscosityOperator {
public:
    void apply(Droplet& drop, double dt) const;
};

class CurvatureFlowOperator {
public:
    void apply(Droplet& drop, const ISurface& surface, double dt) const;
};

class ContactLineOperator {
public:
    void apply(Droplet& drop, const ISurface& surface, double dt, double adhesionDist) const;
};

class VolumeCorrector {
public:
    double computeClosedVolume(const Droplet& drop, const ISurface& surface) const;
    void apply(Droplet& drop, const ISurface& surface, double dt) const;
};

class EdgeLengthRegularizer {
public:
    void apply(Droplet& drop, double dt, double targetRatio, double stiffness, double maxRelativeSpeed) const;
};

// TODO(remesh-step2): Add true topology-changing remeshing operator.
// class AdaptiveRemesher {
// public:
//     void apply(Droplet& drop, double targetLen, double splitThresh, double collapseThresh, int maxOps) const;
// private:
//     bool splitLongestEdge(Droplet& drop, double splitLen) const;
//     bool collapseShortestEdge(Droplet& drop, double collapseLen) const;
//     void rebuildConnectivity(Droplet& drop) const;
// };

} // namespace wd
