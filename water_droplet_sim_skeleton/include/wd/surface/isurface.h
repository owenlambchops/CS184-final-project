#pragma once
#include "wd/core/types.h"

namespace wd {

class ISurface {
public:
    virtual ~ISurface() = default;

    virtual SurfaceSample closestSample(const Vec3& worldPoint) const = 0;
    virtual Vec3 projectPoint(const Vec3& worldPoint) const = 0;
    virtual Vec3 normalAt(const Vec3& worldPoint) const = 0;
    virtual bool raycast(const Ray& ray, PickHit& outHit) const = 0;
    virtual AABB bounds() const = 0;
};

} // namespace wd
