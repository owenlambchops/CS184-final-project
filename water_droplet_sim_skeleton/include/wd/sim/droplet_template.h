#pragma once
#include "wd/core/types.h"

namespace wd {

class DropletTemplate {
public:
    static std::shared_ptr<DropletTemplate> CreateSphericalCap(
        int ringCount, int segmentCount, double baseRadius, double capHeight);

    const MatX3d& restVertices() const { return restV_; }
    const MatX3i& faces() const { return F_; }
    const std::vector<int>& boundaryLoop() const { return boundaryLoop_; }

private:
    MatX3d restV_;
    MatX3i F_;
    std::vector<int> boundaryLoop_;
};

} // namespace wd
