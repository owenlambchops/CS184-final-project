#include "wd/interaction/plane_tilt_interactor.h"
#include "wd/surface/plane_surface.h"

#include <algorithm>
#include <cmath>

namespace wd {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

void PlaneTiltInteractor::update(const InputState& input,
                                 int viewportW,
                                 int viewportH,
                                 double dt,
                                 Scene& scene,
                                 const PlaneTiltParams& params) const {
    if (!params.enabled || !scene.hasSurface() || viewportW <= 0 || viewportH <= 0 || dt <= 0.0) return;

    auto* plane = dynamic_cast<PlaneSurface*>(&scene.surface());
    if (plane == nullptr) return;

    const double xNorm = std::clamp((input.mouseX / static_cast<double>(viewportW)) * 2.0 - 1.0, -1.0, 1.0);
    const double yNorm = std::clamp(1.0 - (input.mouseY / static_cast<double>(viewportH)) * 2.0, -1.0, 1.0);

    const double maxTiltRad = std::clamp(params.maxTiltDeg, 0.0, 30.0) * kPi / 180.0;
    const double slope = std::tan(maxTiltRad);

    const double xComponent = params.axisScaleX * slope * xNorm;
    const double zComponent = params.axisScaleZ * slope * yNorm;

    const Vec3 targetNormal = Vec3(xComponent, 1.0, zComponent).normalized();
    const double response = std::max(params.responsiveness, 0.0);
    const double alpha = 1.0 - std::exp(-response * dt);
    const Vec3 blended = ((1.0 - alpha) * plane->normal() + alpha * targetNormal).normalized();

    if (!std::isfinite(blended.x()) || !std::isfinite(blended.y()) || !std::isfinite(blended.z())) return;
    plane->setNormal(blended);
}

} // namespace wd
