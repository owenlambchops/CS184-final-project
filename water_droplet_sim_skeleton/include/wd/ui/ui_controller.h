#pragma once
#include "wd/core/types.h"
#include "wd/sim/merge_split_controller.h"

namespace wd {

struct UiActions {
    bool createDroplet = false;
    Vec3 spawnAnchor = Vec3::Zero();
    bool applyGravity = false;
    Vec3 gravityForce = Vec3::Zero();
    bool saveScreenshot = false;
    bool setPlaneSideLength = false;
    double planeSideLength = 0.0;
};

class UiController {
public:
    UiActions draw(SolverParams& solverParams,
                   RenderParams& renderParams,
                   MaterialParams& defaultMaterial,
                   SurfaceMaterialParams& surfaceMaterial,
                   SurfaceRenderParams& surfaceRender,
                   double planeSideLength,
                   const Vec3& gravityLikeForce,
                   MergeSplitController& mergeSplit);

private:
    Vec3 spawnAnchor_ = Vec3(0.0, 0.6, 0.0);
    Vec3 gravityDraft_ = Vec3::Zero();
    bool gravityDraftInitialized_ = false;
};

} // namespace wd
