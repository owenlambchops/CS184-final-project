#pragma once
#include "wd/surface/isurface.h"

namespace wd {

class PlaneSurface final : public ISurface {
public:
    PlaneSurface(const Vec3& origin, const Vec3& normal, double sideLength = 6.0);

    SurfaceSample closestSample(const Vec3& worldPoint) const override;
    Vec3 projectPoint(const Vec3& worldPoint) const override;
    Vec3 normalAt(const Vec3& worldPoint) const override;
    bool raycast(const Ray& ray, PickHit& outHit) const override;
    AABB bounds() const override;
    SurfaceRenderMesh buildRenderMesh() const override;

    const SurfaceMaterialParams& material() const override { return material_; }
    SurfaceMaterialParams& material() override { return material_; }
    const SurfaceRenderParams& renderParams() const override { return renderParams_; }
    SurfaceRenderParams& renderParams() override { return renderParams_; }

    const Vec3& origin() const { return origin_; }
    const Vec3& normal() const { return normal_; }
    const Vec3& tangentU() const { return tangentU_; }
    const Vec3& tangentV() const { return tangentV_; }
    double sideLength() const { return sideLength_; }
    void setSideLength(double sideLength);

private:
    bool containsProjection(const Vec3& projectedPoint) const;

    static constexpr double kMinSideLength = 0.1;

    Vec3 origin_;
    Vec3 normal_;
    Vec3 tangentU_;
    Vec3 tangentV_;
    double sideLength_ = 6.0;
    SurfaceMaterialParams material_;
    SurfaceRenderParams renderParams_;
};

} // namespace wd
