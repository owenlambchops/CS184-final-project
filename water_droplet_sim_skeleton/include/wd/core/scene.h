#pragma once
#include "wd/forces/iforce_field.h"
#include "wd/sim/droplet.h"
#include "wd/surface/isurface.h"

namespace wd {

class Scene {
public:
    void setSurface(std::unique_ptr<ISurface> surface);
    bool hasSurface() const;
    const ISurface& surface() const;
    ISurface& surface();

    void setForceField(std::shared_ptr<CompositeForceField> field);
    bool hasForceField() const;
    const IForceField& forceField() const;
    CompositeForceField& compositeForceField();

    std::vector<std::unique_ptr<Droplet>>& droplets();
    const std::vector<std::unique_ptr<Droplet>>& droplets() const;

private:
    std::unique_ptr<ISurface> surface_;
    std::shared_ptr<CompositeForceField> forceField_;
    std::vector<std::unique_ptr<Droplet>> droplets_;
};

} // namespace wd
