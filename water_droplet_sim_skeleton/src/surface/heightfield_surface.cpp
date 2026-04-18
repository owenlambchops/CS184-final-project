#include "wd/surface/heightfield_surface.h"
#include <algorithm>
#include <cmath>

namespace wd {

HeightFieldSurface::HeightFieldSurface(Eigen::MatrixXd heights, Vec3 origin,
                                       double dx, double dz, double heightScale)
    : heights_(std::move(heights)), origin_(origin), dx_(dx), dz_(dz), heightScale_(heightScale) {}

SurfaceSample HeightFieldSurface::closestSample(const Vec3& worldPoint) const {
    double localX = worldPoint.x() - origin_.x();
    double localZ = worldPoint.z() - origin_.z();
    double h = sampleHeight(localX, localZ);

    SurfaceSample s;
    s.position = Vec3(worldPoint.x(), origin_.y() + h, worldPoint.z());
    s.normal = gradientNormal(localX, localZ);
    s.tangentU = Vec3::UnitX();
    s.tangentV = s.normal.cross(s.tangentU);
    if (s.tangentV.norm() < 1e-12) s.tangentV = Vec3::UnitZ();
    s.tangentV.normalize();
    s.tangentU = s.tangentV.cross(s.normal).normalized();
    s.signedDistance = (worldPoint - s.position).dot(s.normal);
    return s;
}

Vec3 HeightFieldSurface::projectPoint(const Vec3& worldPoint) const {
    return closestSample(worldPoint).position;
}

Vec3 HeightFieldSurface::normalAt(const Vec3& worldPoint) const {
    return closestSample(worldPoint).normal;
}

bool HeightFieldSurface::raycast(const Ray& ray, PickHit& outHit) const {
    double t = 0.0;
    for (int i = 0; i < 256; ++i) {
        Vec3 p = ray.origin + t * ray.dir;
        SurfaceSample s = closestSample(p);
        if (std::abs(s.signedDistance) < 1e-3) {
            outHit.hit = true;
            outHit.t = t;
            outHit.position = s.position;
            outHit.normal = s.normal;
            return true;
        }
        t += std::max(1e-3, std::abs(s.signedDistance) * 0.5);
        if (t > 100.0) break;
    }
    return false;
}

AABB HeightFieldSurface::bounds() const {
    AABB b;
    int rows = static_cast<int>(heights_.rows());
    int cols = static_cast<int>(heights_.cols());
    double minH = heights_.size() ? heights_.minCoeff() * heightScale_ : 0.0;
    double maxH = heights_.size() ? heights_.maxCoeff() * heightScale_ : 0.0;
    b.min = origin_ + Vec3(0.0, minH, 0.0);
    b.max = origin_ + Vec3(std::max(0, rows - 1) * dx_, maxH, std::max(0, cols - 1) * dz_);
    return b;
}

double HeightFieldSurface::sampleHeight(double x, double z) const {
    if (heights_.rows() == 0 || heights_.cols() == 0) return 0.0;

    double gx = std::clamp(x / dx_, 0.0, static_cast<double>(heights_.rows() - 1));
    double gz = std::clamp(z / dz_, 0.0, static_cast<double>(heights_.cols() - 1));
    int x0 = static_cast<int>(std::floor(gx));
    int z0 = static_cast<int>(std::floor(gz));
    int x1 = std::min(x0 + 1, static_cast<int>(heights_.rows() - 1));
    int z1 = std::min(z0 + 1, static_cast<int>(heights_.cols() - 1));
    double tx = gx - x0;
    double tz = gz - z0;

    double h00 = heights_(x0, z0);
    double h10 = heights_(x1, z0);
    double h01 = heights_(x0, z1);
    double h11 = heights_(x1, z1);
    double hx0 = (1.0 - tx) * h00 + tx * h10;
    double hx1 = (1.0 - tx) * h01 + tx * h11;
    return heightScale_ * ((1.0 - tz) * hx0 + tz * hx1);
}

Vec3 HeightFieldSurface::gradientNormal(double x, double z) const {
    double epsX = std::max(1e-4, dx_);
    double epsZ = std::max(1e-4, dz_);
    double hL = sampleHeight(x - epsX, z);
    double hR = sampleHeight(x + epsX, z);
    double hD = sampleHeight(x, z - epsZ);
    double hU = sampleHeight(x, z + epsZ);
    double dhdx = (hR - hL) / (2.0 * epsX);
    double dhdz = (hU - hD) / (2.0 * epsZ);
    return Vec3(-dhdx, 1.0, -dhdz).normalized();
}

} // namespace wd
