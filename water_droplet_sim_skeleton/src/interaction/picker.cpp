#include "wd/interaction/picker.h"
#include <cmath>

namespace wd {

Ray Picker::makeCameraRay(double mouseX, double mouseY, int viewportW, int viewportH, const Camera& camera) const {
    Ray ray;
    ray.origin = camera.position();

    if (viewportW <= 0 || viewportH <= 0) {
        ray.dir = camera.forward();
        return ray;
    }

    const double ndcX = 2.0 * mouseX / static_cast<double>(viewportW) - 1.0;
    const double ndcY = 1.0 - 2.0 * mouseY / static_cast<double>(viewportH);
    const double aspect = static_cast<double>(viewportW) / static_cast<double>(viewportH);
    const double tanHalfFovY = std::tan(0.5 * camera.fovYRad());

    Vec3 dir = camera.forward()
        + ndcX * aspect * tanHalfFovY * camera.right()
        + ndcY * tanHalfFovY * camera.up();

    ray.dir = dir.norm() > 1e-8 ? dir.normalized() : camera.forward();
    return ray;
}

PickHit Picker::pickSurface(const Ray& ray, const ISurface& surface) const {
    PickHit hit;
    surface.raycast(ray, hit);
    return hit;
}

} // namespace wd
