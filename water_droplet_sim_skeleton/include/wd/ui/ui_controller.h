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
    bool setMeshSubdivisions = false;
    int meshSubdivisions = 2;
    bool disable_tilt = true;

    // Plane tilt  ← NEW
    bool tiltPlane      = false;
    Vec3 planeNormal    = Vec3::UnitY();
};

class UiController {
public:
    UiActions draw(SolverParams&          solverParams,
                   RenderParams&          renderParams,
                   MaterialParams&        defaultMaterial,
                   const Vec3&            gravityLikeForce,
                   MergeSplitController&  mergeSplit);
private:
    Vec3   spawnAnchor_ = Vec3::Zero();
    Vec3   gravityDraft_ = Vec3::Zero();
    bool   gravityDraftInitialized_ = false;

    // Plane tilt  ← NEW
    double pitchDeg_ = 0.0;
    double rollDeg_  = 0.0;
    static constexpr double kTiltMin = -80.0;
    static constexpr double kTiltMax =  80.0;
};

// namespace wd
};