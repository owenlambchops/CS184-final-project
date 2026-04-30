#pragma once
#include "wd/sim/droplet_cache.h"
#include "wd/sim/droplet_template.h"

namespace wd {

class Droplet {
public:
    Droplet(int id, std::shared_ptr<const DropletTemplate> tpl, const MaterialParams& material);

    int id() const { return id_; }

    const MatX3i& faces() const { return tpl_->faces(); }
    const std::vector<int>& boundaryLoop() const { return tpl_->boundaryLoop(); }

    const MatX3d& positions() const { return X_; }
    MatX3d& positions() { return X_; }

    const MatX3d& velocities() const { return U_; }
    MatX3d& velocities() { return U_; }

    const MaterialParams& material() const { return material_; }
    MaterialParams& material() { return material_; }

    double targetVolume() const { return targetVolume_; }
    void setTargetVolume(double v) { targetVolume_ = v; }

    const DropletDerivedData& derived() const { return derived_; }
    DropletDerivedData& derived() { return derived_; }

    void updateDerived();

private:
    int id_;
    std::shared_ptr<const DropletTemplate> tpl_;
    MatX3d X_;
    MatX3d U_;
    MaterialParams material_;
    double targetVolume_ = 0.0;
    DropletDerivedData derived_;
};

} // namespace wd
