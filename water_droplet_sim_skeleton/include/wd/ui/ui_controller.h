#pragma once
#include "wd/core/types.h"
#include "wd/sim/merge_split_controller.h"

namespace wd {

struct UiActions {
    bool createDroplet = false;
    Vec3 spawnAnchor = Vec3::Zero();
    bool applyGravity = false;
    Vec3 gravityForce = Vec3(0.0, -9.81, 0.0);
    bool saveScreenshot = false;
    bool setPlaneSideLength = false;
    double planeSideLength = 0.0;
    bool setPlaneTiltEnabled = false;
    bool planeTiltEnabled = true;
    bool setPlaneTiltMaxDeg = false;
    double planeTiltMaxDeg = 15.0;
    bool setPlaneTiltResponsiveness = false;
    double planeTiltResponsiveness = 10.0;
    bool setPlaneTiltAxisScaleX = false;
    double planeTiltAxisScaleX = 1.0;
    bool setPlaneTiltAxisScaleZ = false;
    double planeTiltAxisScaleZ = 1.0;
    bool resetPlaneAndDisableInteraction = false;
};

class UiController {
public:
    UiActions draw(SolverParams& solverParams,
                   RenderParams& renderParams,
                   MaterialParams& defaultMaterial,
                   SurfaceMaterialParams& surfaceMaterial,
                   SurfaceRenderParams& surfaceRender,
                   double planeSideLength,
                   bool planeTiltEnabled,
                   double planeTiltMaxDeg,
                   double planeTiltResponsiveness,
                   double planeTiltAxisScaleX,
                   double planeTiltAxisScaleZ,
                   const Vec3& gravityLikeForce,
                   MergeSplitController& mergeSplit);

private:
    Vec3 spawnAnchor_ = Vec3(0.0, 0.6, 0.0);
    Vec3 gravityDraft_ = Vec3(0.0, -9.81, 0.0);
    bool gravityDraftInitialized_ = true;
};

} // namespace wd
