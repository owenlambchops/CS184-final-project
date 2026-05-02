#pragma once
#include "wd/core/types.h"
#include "wd/sim/merge_split_controller.h"

namespace wd {

struct UiActions {
    // Droplet spawning
    bool createDroplet  = false;
    Vec3 spawnAnchor    = Vec3::Zero();

    // Gravity
    bool applyGravity   = false;
    Vec3 gravityForce   = Vec3::Zero();

    // Screenshot
    bool saveScreenshot = false;

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