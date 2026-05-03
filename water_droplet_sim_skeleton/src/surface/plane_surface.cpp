#include "wd/surface/plane_surface.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace wd {
namespace {
Vec3 safeNormalize(const Vec3& v, const Vec3& fallback) {
    double n = v.norm();
    return n > 1e-12 ? v / n : fallback;
}

void buildPlaneFrame(const Vec3& normal, Vec3& tangentU, Vec3& tangentV) {
    Vec3 seed = std::abs(normal.dot(Vec3::UnitY())) < 0.9 ? Vec3::UnitY() : Vec3::UnitX();
    tangentU = safeNormalize(seed.cross(normal), Vec3::UnitX());
    tangentV = safeNormalize(normal.cross(tangentU), Vec3::UnitZ());
}

constexpr double kOutsideSignedDistance = 1e6;
} // namespace

PlaneSurface::PlaneSurface(const Vec3& origin, const Vec3& normal, double sideLength)
    : origin_(origin), normal_(safeNormalize(normal, Vec3::UnitY())) {
    buildPlaneFrame(normal_, tangentU_, tangentV_);
    setSideLength(sideLength);
}

SurfaceSample PlaneSurface::closestSample(const Vec3& worldPoint) const {
    SurfaceSample s;
    double d = (worldPoint - origin_).dot(normal_);
    Vec3 projected = worldPoint - d * normal_;
    s.position = projected;
    s.normal = normal_;
    s.tangentU = tangentU_;
    s.tangentV = tangentV_;
    s.signedDistance = containsProjection(projected) ? d : kOutsideSignedDistance;
    return s;
}

Vec3 PlaneSurface::projectPoint(const Vec3& worldPoint) const {
    return closestSample(worldPoint).position;
}

Vec3 PlaneSurface::normalAt(const Vec3&) const { return normal_; }

bool PlaneSurface::raycast(const Ray& ray, PickHit& outHit) const {
    double denom = ray.dir.dot(normal_);
    if (std::abs(denom) < 1e-10) return false;
    double t = (origin_ - ray.origin).dot(normal_) / denom;
    if (t < 0.0) return false;

    Vec3 hitPosition = ray.origin + t * ray.dir;
    if (!containsProjection(hitPosition)) return false;

    outHit.hit = true;
    outHit.t = t;
    outHit.position = hitPosition;
    outHit.normal = normal_;
    return true;
}

AABB PlaneSurface::bounds() const {
    AABB b;
    const double half = 0.5 * sideLength_;
    const Vec3 u = half * tangentU_;
    const Vec3 v = half * tangentV_;
    const std::array<Vec3, 4> corners = {
        origin_ - u - v,
        origin_ + u - v,
        origin_ + u + v,
        origin_ - u + v,
    };

    for (const Vec3& corner : corners) {
        b.min = b.min.cwiseMin(corner);
        b.max = b.max.cwiseMax(corner);
    }
    return b;
}

SurfaceRenderMesh PlaneSurface::buildRenderMesh() const {
    SurfaceRenderMesh mesh;
    mesh.positions.resize(4, 3);
    mesh.normals.resize(4, 3);
    mesh.faces.resize(2, 3);

    const double half = 0.5 * sideLength_;
    const Vec3 u = half * tangentU_;
    const Vec3 v = half * tangentV_;
    mesh.positions.row(0) = (origin_ - u - v).transpose();
    mesh.positions.row(1) = (origin_ + u - v).transpose();
    mesh.positions.row(2) = (origin_ + u + v).transpose();
    mesh.positions.row(3) = (origin_ - u + v).transpose();

    for (int i = 0; i < mesh.normals.rows(); ++i) {
        mesh.normals.row(i) = normal_.transpose();
    }

    mesh.faces << 0, 1, 2,
                  0, 2, 3;
    return mesh;
}

void PlaneSurface::setSideLength(double sideLength) {
    sideLength_ = std::max(kMinSideLength, sideLength);
}

void PlaneSurface::setNormal(const Vec3& normal) {
    normal_ = safeNormalize(normal, Vec3::UnitY());
    buildPlaneFrame(normal_, tangentU_, tangentV_);
}

bool PlaneSurface::containsProjection(const Vec3& projectedPoint) const {
    const Vec3 local = projectedPoint - origin_;
    const double half = 0.5 * sideLength_;
    constexpr double eps = 1e-8;
    return std::abs(local.dot(tangentU_)) <= half + eps &&
           std::abs(local.dot(tangentV_)) <= half + eps;
}

} // namespace wd
