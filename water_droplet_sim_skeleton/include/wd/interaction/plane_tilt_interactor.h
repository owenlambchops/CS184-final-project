#pragma once
#include "wd/core/scene.h"
#include "wd/interaction/input_router.h"

namespace wd {

struct PlaneTiltParams {
    bool enabled = true;
    double maxTiltDeg = 15.0;
    double responsiveness = 10.0;
    double axisScaleX = 1.0;
    double axisScaleZ = 1.0;
};

class PlaneTiltInteractor {
public:
    void update(const InputState& input,
                int viewportW,
                int viewportH,
                double dt,
                Scene& scene,
                const PlaneTiltParams& params) const;
};

} // namespace wd
