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

    // Map cursor from screen coordinates into centered NDC-like range [-1, 1].
    // xNorm = -1 at left edge, +1 at right edge.
    const double xNorm = std::clamp((input.mouseX / (static_cast<double>(viewportW)/2.0)) * 2.0 - 1.0, -1.0, 1.0);
    // yNorm is inverted so +1 means top of screen, matching intuitive "tilt up".
    const double yNorm = std::clamp((input.mouseY / (static_cast<double>(viewportH)/2.0)) * 2.0 - 1.0, -1.0, 1.0);

    // Convert max tilt angle to slope (tan(theta)). This sets horizontal normal magnitude.
    const double maxTiltRad = std::clamp(params.maxTiltDeg, 0.0, 30.0) * kPi / 180.0;
    const double slope = std::tan(maxTiltRad);

    // Project normalized cursor offsets into world X/Z normal components.
    const double xComponent = params.axisScaleX * xNorm;
    const double zComponent = params.axisScaleZ * yNorm;

    // Build target normal with fixed positive Y, then smooth toward it over time.
    const Vec3 targetNormal = Vec3(xComponent, 1.0, zComponent).normalized();
    const double response = std::max(params.responsiveness, 0.0);
    const double alpha = 1.0 - std::exp(-response * dt);
    const Vec3 blended = ((1.0 - alpha) * plane->normal() + alpha * targetNormal).normalized();
}

} // namespace wd
