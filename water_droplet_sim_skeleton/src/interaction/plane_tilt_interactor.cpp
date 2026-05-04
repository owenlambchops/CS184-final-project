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

    // Map cursor into centered range [-1, 1].
    // xNorm = -1 at left edge, +1 at right edge.
    const double xNorm = std::clamp(2.0 * (input.mouseX / viewportW) - 1.0, -1.0, 1.0);
    // yNorm = +1 at top edge (tilt away), -1 at bottom edge (tilt towards screen).
    const double yNorm = std::clamp(2.0 * (input.mouseY / viewportH) - 1.0, -1.0, 1.0);

    // Convert max tilt angle to slope (tan(theta)). This sets horizontal normal magnitude and responsiveness.
    // const double maxTiltRad = std::clamp(params.maxTiltDeg, 0.0, 30.0) * kPi / 180.0;
    // const double slope = std::tan(maxTiltRad);

    // Project normalized cursor offsets into world X/Z normal components, scaled by slope for responsiveness.
    const double xComponent = params.axisScaleX * xNorm * 1.5;
    const double zComponent = params.axisScaleZ * yNorm * 1.5;

    // Build target normal with fixed positive Y, then smooth toward it over time.
    const Vec3 targetNormal = Vec3(xComponent, 1.0, zComponent).normalized();
    const double response = std::max(params.responsiveness, 0.0);
    const double alpha = 1.0 - std::exp(-response * dt);
    const Vec3 blended = ((1.0 - alpha) * plane->normal() + alpha * targetNormal).normalized();
    plane->setNormal(blended);
}

} // namespace wd
