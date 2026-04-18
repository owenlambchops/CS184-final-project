#include "wd/surface/plane_surface.h"
#include <cmath>

namespace wd {
namespace {
Vec3 safeNormalize(const Vec3& v, const Vec3& fallback) {
    double n = v.norm();
    return n > 1e-12 ? v / n : fallback;
}
} // namespace

PlaneSurface::PlaneSurface(const Vec3& origin, const Vec3& normal)
    : origin_(origin), normal_(safeNormalize(normal, Vec3::UnitY())) {
    Vec3 seed = std::abs(normal_.dot(Vec3::UnitY())) < 0.9 ? Vec3::UnitY() : Vec3::UnitX();
    tangentU_ = safeNormalize(seed.cross(normal_), Vec3::UnitX());
    tangentV_ = safeNormalize(normal_.cross(tangentU_), Vec3::UnitZ());
}

SurfaceSample PlaneSurface::closestSample(const Vec3& worldPoint) const {
    SurfaceSample s;
    double d = (worldPoint - origin_).dot(normal_);
    s.position = worldPoint - d * normal_;
    s.normal = normal_;
    s.tangentU = tangentU_;
    s.tangentV = tangentV_;
    s.signedDistance = d;
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

    outHit.hit = true;
    outHit.t = t;
    outHit.position = ray.origin + t * ray.dir;
    outHit.normal = normal_;
    return true;
}

AABB PlaneSurface::bounds() const {
    AABB b;
    b.min = Vec3(-10.0, -10.0, -10.0);
    b.max = Vec3( 10.0,  10.0,  10.0);
    return b;
}

} // namespace wd
