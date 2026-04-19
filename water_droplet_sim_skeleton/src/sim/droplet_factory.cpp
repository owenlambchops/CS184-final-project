#include "wd/sim/droplet_factory.h"
#include <algorithm>

namespace wd {

DropletFactory::DropletFactory(std::shared_ptr<const DropletTemplate> tpl) : tpl_(std::move(tpl)) {}

std::unique_ptr<Droplet> DropletFactory::spawn(int id, const SpawnDesc& desc, const ISurface& surface) const {
    auto drop = std::make_unique<Droplet>(id, tpl_, desc.material);
    SurfaceSample s = surface.closestSample(desc.anchorWorld);

    auto& X = drop->positions();
    auto& U = drop->velocities();

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 local = tpl_->restVertices().row(i).transpose();
        Vec3 world = s.position + local.x() * s.tangentU + local.y() * s.normal + local.z() * s.tangentV;
        X.row(i) = world.transpose();
        U.row(i) = desc.initialVelocity.transpose();
    }

    drop->setTargetVolume(desc.targetVolume);
    drop->updateDerived();
    return drop;
}

std::unique_ptr<Droplet> DropletFactory::spawnMerged(
    int id, const Droplet& a, const Droplet& b, const ISurface& surface, double damping) const {
    double va = std::max(a.targetVolume(), 1e-12);
    double vb = std::max(b.targetVolume(), 1e-12);
    double total = va + vb;

    SpawnDesc desc;
    desc.anchorWorld = (va * a.derived().centerOfMass + vb * b.derived().centerOfMass) / total;
    desc.initialVelocity = damping * (va * a.derived().avgVelocity + vb * b.derived().avgVelocity) / total;
    desc.targetVolume = total;
    desc.material = a.material();
    return spawn(id, desc, surface);
}

std::pair<std::unique_ptr<Droplet>, std::unique_ptr<Droplet>> DropletFactory::spawnSplit(
    int id0, int id1, const Droplet& src, const ISurface& surface, double volumeRatio) const {
    volumeRatio = std::clamp(volumeRatio, 0.1, 0.9);

    Vec3 axis = Vec3::UnitX();
    if (src.derived().principalAxis.norm() > 1e-8) {
        axis = src.derived().principalAxis.normalized();
    }
    double offset = std::max(0.05, 0.35 * src.derived().footprintRadius);

    SpawnDesc a;
    a.anchorWorld = src.derived().centerOfMass - offset * axis;
    a.initialVelocity = src.derived().avgVelocity;
    a.targetVolume = src.targetVolume() * volumeRatio;
    a.material = src.material();

    SpawnDesc b = a;
    b.anchorWorld = src.derived().centerOfMass + offset * axis;
    b.targetVolume = src.targetVolume() - a.targetVolume;

    return {spawn(id0, a, surface), spawn(id1, b, surface)};
}

} // namespace wd
