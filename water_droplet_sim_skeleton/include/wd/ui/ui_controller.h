#pragma once
#include "wd/core/types.h"
#include "wd/sim/merge_split_controller.h"

namespace wd {

class UiController {
public:
    void draw(SolverParams& solverParams,
              RenderParams& renderParams,
              MaterialParams& defaultMaterial,
              Vec3& gravityLikeForce,
              MergeSplitController& mergeSplit);
};

} // namespace wd
