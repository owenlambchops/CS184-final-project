#pragma once
#include "wd/surface/isurface.h"

namespace wd {

class PlaneSurface final : public ISurface {
public:
    PlaneSurface(const Vec3& origin, const Vec3& normal);

    void setNormal(const Vec3& normal) {
        normal_ = safeNormalize(normal, Vec3::UnitY());
        // Recompute tangent frame (same logic as constructor).
        Vec3 seed = std::abs(normal_.dot(Vec3::UnitY())) < 0.9
                        ? Vec3::UnitY() : Vec3::UnitX();
        tangentU_ = safeNormalize(seed.cross(normal_), Vec3::UnitX());
        tangentV_ = safeNormalize(normal_.cross(tangentU_), Vec3::UnitZ());
    }
 
    void setOrigin(const Vec3& origin) { origin_ = origin; }

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
    void setNormal(const Vec3& normal);
    void setSideLength(double sideLength);

private:

    static Vec3 safeNormalize(const Vec3& v, const Vec3& fallback) {
        double n = v.norm();
        return n > 1e-12 ? v / n : fallback;
    }
    
    Vec3 origin_;
    Vec3 normal_;
    Vec3 tangentU_;
    Vec3 tangentV_;
    double sideLength_ = 6.0;
    SurfaceMaterialParams material_;
    SurfaceRenderParams renderParams_;
};

} // namespace wd
