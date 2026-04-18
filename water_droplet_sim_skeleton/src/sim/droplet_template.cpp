#include "wd/sim/droplet_template.h"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace wd {

std::shared_ptr<DropletTemplate> DropletTemplate::CreateSphericalCap(
    int ringCount, int segmentCount, double baseRadius, double capHeight) {
    ringCount = std::max(2, ringCount);
    segmentCount = std::max(8, segmentCount);

    auto tpl = std::make_shared<DropletTemplate>();
    int vertexCount = 1 + ringCount * segmentCount;
    tpl->restV_.resize(vertexCount, 3);

    tpl->restV_.row(0) = Vec3(0.0, capHeight, 0.0).transpose();

    for (int r = 1; r <= ringCount; ++r) {
        double u = static_cast<double>(r) / static_cast<double>(ringCount);
        double rad = baseRadius * u;
        double y = capHeight * (1.0 - u * u);
        for (int s = 0; s < segmentCount; ++s) {
            double a = 2.0 * std::numbers::pi * static_cast<double>(s) / static_cast<double>(segmentCount);
            int id = 1 + (r - 1) * segmentCount + s;
            tpl->restV_.row(id) = Vec3(rad * std::cos(a), y, rad * std::sin(a)).transpose();
        }
    }

    int faceCount = segmentCount + (ringCount - 1) * segmentCount * 2;
    tpl->F_.resize(faceCount, 3);
    int f = 0;

    for (int s = 0; s < segmentCount; ++s) {
        int a = 1 + s;
        int b = 1 + ((s + 1) % segmentCount);
        tpl->F_.row(f++) = Eigen::RowVector3i(0, a, b);
    }

    for (int r = 1; r < ringCount; ++r) {
        int curr = 1 + (r - 1) * segmentCount;
        int next = 1 + r * segmentCount;
        for (int s = 0; s < segmentCount; ++s) {
            int c0 = curr + s;
            int c1 = curr + ((s + 1) % segmentCount);
            int n0 = next + s;
            int n1 = next + ((s + 1) % segmentCount);
            tpl->F_.row(f++) = Eigen::RowVector3i(c0, n0, n1);
            tpl->F_.row(f++) = Eigen::RowVector3i(c0, n1, c1);
        }
    }

    int boundaryStart = 1 + (ringCount - 1) * segmentCount;
    for (int s = 0; s < segmentCount; ++s) tpl->boundaryLoop_.push_back(boundaryStart + s);
    return tpl;
}

} // namespace wd
