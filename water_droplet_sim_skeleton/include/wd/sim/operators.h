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
    void apply(Droplet& drop, const ISurface& surface, double pushoutEps) const;
};

class ViscosityOperator {
public:
    void apply(Droplet& drop, double dt) const;
};

class CurvatureFlowOperator {
public:
    void apply(Droplet& drop, double dt) const;
};

class ContactLineOperator {
public:
    void apply(Droplet& drop, const ISurface& surface, double dt) const;
};

class VolumeCorrector {
public:
    double computeClosedVolume(const Droplet& drop, const ISurface& surface) const;
    void apply(Droplet& drop, const ISurface& surface, double dt) const;
};

} // namespace wd
