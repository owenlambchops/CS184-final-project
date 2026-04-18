#pragma once
#include "wd/surface/isurface.h"

namespace wd {

class PlaneSurface final : public ISurface {
public:
    PlaneSurface(const Vec3& origin, const Vec3& normal);

    SurfaceSample closestSample(const Vec3& worldPoint) const override;
    Vec3 projectPoint(const Vec3& worldPoint) const override;
    Vec3 normalAt(const Vec3& worldPoint) const override;
    bool raycast(const Ray& ray, PickHit& outHit) const override;
    AABB bounds() const override;

    const Vec3& origin() const { return origin_; }
    const Vec3& normal() const { return normal_; }
    const Vec3& tangentU() const { return tangentU_; }
    const Vec3& tangentV() const { return tangentV_; }

private:
    Vec3 origin_;
    Vec3 normal_;
    Vec3 tangentU_;
    Vec3 tangentV_;
};

} // namespace wd
