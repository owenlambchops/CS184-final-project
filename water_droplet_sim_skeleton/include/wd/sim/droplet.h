#pragma once
#include "wd/sim/droplet_cache.h"
#include "wd/sim/droplet_template.h"

namespace wd {

class Droplet {
public:
    Droplet(int id, std::shared_ptr<const DropletTemplate> tpl, const MaterialParams& material);

    int id() const { return id_; }

    const MatX3i& faces() const { return F_; }
    MatX3i& faces() { return F_; }
    const std::vector<Eigen::Vector2i>& edges() const { return E_; }
    std::vector<Eigen::Vector2i>& edges() { return E_; }
    const std::vector<int>& boundaryLoop() const { return boundaryLoop_; }

    const MatX3d& positions() const { return X_; }
    MatX3d& positions() { return X_; }

    const MatX3d& velocities() const { return U_; }
    MatX3d& velocities() { return U_; }

    const MaterialParams& material() const { return material_; }
    MaterialParams& material() { return material_; }

    double targetVolume() const { return targetVolume_; }
    void setTargetVolume(double v) { targetVolume_ = v; }
    int initialVertexCount() const { return initialVertexCount_; }

    const DropletDerivedData& derived() const { return derived_; }
    DropletDerivedData& derived() { return derived_; }

    void updateDerived();
    // TODO(remesh-step2): Add runtime topology helpers used by split/collapse remeshing.
    // - void rebuildEdgesFromFaces();
    // - std::vector<std::vector<int>> buildVertexFaceAdjacency() const;
    // - std::vector<std::vector<int>> buildVertexVertexAdjacency() const;
    // - bool validateManifoldTopology() const;
    // - void removeDegenerateFaces(double areaEps);

private:
    int id_;
    std::shared_ptr<const DropletTemplate> tpl_;
    MatX3d X_;
    MatX3d U_;
    MatX3i F_;
    std::vector<Eigen::Vector2i> E_;
    std::vector<int> boundaryLoop_;
    MaterialParams material_;
    double targetVolume_ = 0.0;
    int initialVertexCount_ = 0;
    DropletDerivedData derived_;
};

} // namespace wd
