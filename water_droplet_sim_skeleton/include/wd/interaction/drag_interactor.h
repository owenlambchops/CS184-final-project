#pragma once
#include "wd/core/scene.h"
#include "wd/forces/iforce_field.h"
#include "wd/interaction/input_router.h"
#include "wd/interaction/picker.h"

namespace wd {

class DragInteractor {
public:
    explicit DragInteractor(std::shared_ptr<DragForceField> dragField);

    void update(const InputState& input, const Camera& camera, int viewportW, int viewportH, Scene& scene);

private:
    std::shared_ptr<DragForceField> dragField_;
    Picker picker_;
};

} // namespace wd
