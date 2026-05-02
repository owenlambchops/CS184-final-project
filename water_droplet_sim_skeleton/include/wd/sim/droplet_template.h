#pragma once
#include "wd/core/types.h"

namespace wd {

class DropletTemplate {
public:
    static std::shared_ptr<DropletTemplate> CreateSphericalMesh(int subdivisions, double radius);

    const MatX3d& restVertices() const { return restV_; }
    const MatX3i& faces() const { return F_; }
    const std::vector<Eigen::Vector2i>& edges() const { return E_; }
    const std::vector<int>& boundaryLoop() const { return boundaryLoop_; }

private:
    MatX3d restV_;
    MatX3i F_;
    std::vector<Eigen::Vector2i> E_;
    std::vector<int> boundaryLoop_;
};

} // namespace wd
