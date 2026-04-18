#pragma once
#include "wd/sim/droplet.h"
#include "wd/surface/isurface.h"

namespace wd {

struct SpawnDesc {
    Vec3 anchorWorld = Vec3::Zero();
    Vec3 initialVelocity = Vec3::Zero();
    double targetVolume = 0.01;
    MaterialParams material;
};

class DropletFactory {
public:
    explicit DropletFactory(std::shared_ptr<const DropletTemplate> tpl);

    std::unique_ptr<Droplet> spawn(int id, const SpawnDesc& desc, const ISurface& surface) const;

    std::unique_ptr<Droplet> spawnMerged(
        int id, const Droplet& a, const Droplet& b, const ISurface& surface, double damping = 0.8) const;

    std::pair<std::unique_ptr<Droplet>, std::unique_ptr<Droplet>> spawnSplit(
        int id0, int id1, const Droplet& src, const ISurface& surface, double volumeRatio = 0.5) const;

private:
    std::shared_ptr<const DropletTemplate> tpl_;
};

} // namespace wd
