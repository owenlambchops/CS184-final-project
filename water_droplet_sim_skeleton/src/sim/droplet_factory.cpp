#include "wd/sim/droplet_factory.h"
#include "wd/sim/cgl_math.h"
#include <algorithm>

namespace wd {

DropletFactory::DropletFactory(std::shared_ptr<const DropletTemplate> tpl) : tpl_(std::move(tpl)) {}

std::unique_ptr<Droplet> DropletFactory::spawn(int id, const SpawnDesc& desc, const ISurface&) const {
    auto drop = std::make_unique<Droplet>(id, tpl_, desc.material);

    auto& X = drop->positions();
    auto& U = drop->velocities();

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 local = tpl_->restVertices().row(i).transpose();
        Vec3 world = desc.anchorWorld + local;
        X.row(i) = world.transpose();
        U.row(i) = desc.initialVelocity.transpose();
    }

    drop->updateDerived();
    const double targetVolume = (desc.targetVolume > 0.0) ? desc.targetVolume : drop->derived().currentVolume;
    drop->setTargetVolume(targetVolume);
    return drop;
}

std::unique_ptr<Droplet> DropletFactory::spawnMerged(
    int id, const Droplet& a, const Droplet& b, const ISurface& surface, double damping) const {
    double va = std::max(a.targetVolume(), 1e-12);
    double vb = std::max(b.targetVolume(), 1e-12);
    double total = va + vb;
    double wa = va / total;
    double wb = vb / total;

    // Create base droplet from template for structure
    SpawnDesc baseDesc;
    baseDesc.material = a.material();
    auto merged = spawn(id, baseDesc, surface);

    // Extract mesh data from both droplets
    const auto& X_a = a.positions();
    const auto& X_b = b.positions();
    const auto& U_a = a.velocities();
    const auto& U_b = b.velocities();
    const auto& F_a = a.faces();
    const auto& F_b = b.faces();

    // Try to compute geometric boolean union via CGAL. If it fails, fall back to simple concatenation.
    MatX3d unionX;
    MatX3i unionF;
    bool unionOk = booleanUnionMeshes(X_a, F_a, X_b, F_b, unionX, unionF);

    if (unionOk && unionX.rows() > 0 && unionF.rows() > 0) {
        // Assign union mesh
        merged->positions() = unionX;
        // Set velocities to weighted average per-droplet velocity (flat per-vertex approximation)
        MatX3d U_merged = MatX3d::Zero(unionX.rows(), 3);
        const Vec3 avgVel = (wa * a.derived().avgVelocity + wb * b.derived().avgVelocity) * damping;
        for (int i = 0; i < unionX.rows(); ++i) U_merged.row(i) = avgVel.transpose();
        merged->velocities() = U_merged;
        merged->faces() = unionF;
        merged->edges() = buildUniqueEdgesFromFaces(unionF);
    } else {
        // Fallback: concatenate vertex/face lists
        int nv_a = static_cast<int>(X_a.rows());
        int nv_b = static_cast<int>(X_b.rows());
        int nv_total = nv_a + nv_b;

        MatX3d X_merged(nv_total, 3);
        MatX3d U_merged(nv_total, 3);

        X_merged.topRows(nv_a) = X_a;
        X_merged.bottomRows(nv_b) = X_b;

        // Velocities: weighted interpolation when possible, otherwise copy average
        const Vec3 avgVel = (wa * a.derived().avgVelocity + wb * b.derived().avgVelocity) * damping;
        for (int i = 0; i < nv_total; ++i) U_merged.row(i) = avgVel.transpose();

        int nf_a = static_cast<int>(F_a.rows());
        int nf_b = static_cast<int>(F_b.rows());
        MatX3i F_merged(nf_a + nf_b, 3);
        F_merged.topRows(nf_a) = F_a;
        for (int i = 0; i < nf_b; ++i) {
            F_merged.row(nf_a + i) = F_b.row(i).array() + nv_a;
        }

        merged->positions() = X_merged;
        merged->velocities() = U_merged;
        merged->faces() = F_merged;
        merged->edges() = buildUniqueEdgesFromFaces(F_merged);
    }

    // Set target volume to sum
    merged->setTargetVolume(total);

    // Recompute all derived geometric data
    merged->updateDerived();

    return merged;
}

std::pair<std::unique_ptr<Droplet>, std::unique_ptr<Droplet>> DropletFactory::spawnSplit(
    int id0, int id1, const Droplet& src, const ISurface& surface, double volumeRatio) const {
    volumeRatio = std::clamp(volumeRatio, 0.1, 0.9);

    Vec3 axis = Vec3::UnitX();
    if (src.derived().principalAxis.norm() > 1e-8) {
        axis = src.derived().principalAxis.normalized();
    }
    double offset = std::max(0.05, src.derived().footprintRadius);

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
