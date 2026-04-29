#pragma once
#include "wd/core/types.h"
#include "wd/sim/merge_split_controller.h"

namespace wd {

struct UiActions {
    bool createDroplet = false;
    Vec3 spawnAnchor = Vec3::Zero();
    bool applyGravity = false;
    Vec3 gravityForce = Vec3::Zero();
};

class UiController {
public:
    UiActions draw(SolverParams& solverParams,
                   RenderParams& renderParams,
                   MaterialParams& defaultMaterial,
                   SurfaceMaterialParams& surfaceMaterial,
                   SurfaceRenderParams& surfaceRender,
                   const Vec3& gravityLikeForce,
                   MergeSplitController& mergeSplit);

private:
    Vec3 spawnAnchor_ = Vec3::Zero();
    Vec3 gravityDraft_ = Vec3::Zero();
    bool gravityDraftInitialized_ = false;
};

} // namespace wd
