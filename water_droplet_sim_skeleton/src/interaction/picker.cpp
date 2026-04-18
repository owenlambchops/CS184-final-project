#include "wd/interaction/picker.h"

namespace wd {

Ray Picker::makeCameraRay(double, double, int, int, const Camera& camera) const {
    // TODO: replace with inverse-projection/inverse-view unprojection.
    Ray ray;
    ray.origin = camera.position();
    ray.dir = Vec3(0.0, -0.25, -1.0).normalized();
    return ray;
}

PickHit Picker::pickSurface(const Ray& ray, const ISurface& surface) const {
    PickHit hit;
    surface.raycast(ray, hit);
    return hit;
}

} // namespace wd
