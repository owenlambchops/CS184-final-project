#pragma once
#include "wd/render/refractive_renderer.h"
#include "wd/surface/isurface.h"

namespace wd {

class Picker {
public:
    Ray makeCameraRay(double mouseX, double mouseY, int viewportW, int viewportH, const Camera& camera) const;
    PickHit pickSurface(const Ray& ray, const ISurface& surface) const;
};

} // namespace wd
