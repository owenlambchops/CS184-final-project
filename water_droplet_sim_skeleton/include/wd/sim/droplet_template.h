#pragma once
#include "wd/core/types.h"

namespace wd {

class DropletTemplate {
public:
    static std::shared_ptr<DropletTemplate> CreatePolyhedron(
        int nVertices, double radius);

    const MatX3d& restVertices() const { return restV_; }
    const MatX3i& faces() const { return F_; }

private:
    MatX3d restV_;
    MatX3i F_;
};

} // namespace wd
