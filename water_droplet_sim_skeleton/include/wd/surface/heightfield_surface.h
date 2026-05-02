#pragma once
#include "wd/surface/isurface.h"

namespace wd {

class HeightFieldSurface final : public ISurface {
public:
    HeightFieldSurface(Eigen::MatrixXd heights, Vec3 origin, double dx, double dz, double heightScale);

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

private:
    double sampleHeight(double x, double z) const;
    Vec3 gradientNormal(double x, double z) const;

    Eigen::MatrixXd heights_;
    Vec3 origin_;
    double dx_ = 1.0;
    double dz_ = 1.0;
    double heightScale_ = 1.0;
    SurfaceMaterialParams material_;
    SurfaceRenderParams renderParams_;
};

} // namespace wd
