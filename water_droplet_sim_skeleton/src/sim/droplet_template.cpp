#include "wd/sim/droplet_template.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <igl/convex_hull.h>

namespace wd {

namespace {
// Generates uniformly distributed points on sphere using Fibonacci sphere algorithm
MatX3d generatePointsOnSphere(int nPoints, double radius) {
    MatX3d points(nPoints, 3);
    double goldenRatio = (1.0 + std::sqrt(5.0)) / 2.0;
    for (int i = 0; i < nPoints; ++i) {
        double theta = 2.0 * std::numbers::pi * i / goldenRatio;
        double phi = std::acos(1.0 - 2.0 * i / static_cast<double>(nPoints));
        double x = std::cos(theta) * std::sin(phi);
        double y = std::sin(theta) * std::sin(phi);
        double z = std::cos(phi);
        points.row(i) = Vec3(x, y, z) * radius;
    }
    return points;
}
} // namespace

std::shared_ptr<DropletTemplate> DropletTemplate::CreatePolyhedron(
    int nVertices, double radius) {
    
    if (nVertices < 4) nVertices = 4;
    
    auto tpl = std::make_shared<DropletTemplate>();
    
    // Generate points uniformly on sphere
    MatX3d spherePoints = generatePointsOnSphere(nVertices, radius);

    // Use libigl convex hull for watertight triangulation
    MatX3i hullFaces;
    Eigen::VectorXi J;
    igl::convex_hull(spherePoints, hullFaces, J);

    tpl->restV_ = spherePoints;
    tpl->F_ = hullFaces;
    return tpl;
}

} // namespace wd
