#pragma once
// wd/forces/droplet_drag_force_field.h
//
// A force field that pulls a single targeted droplet's vertices
// toward a world-space grab point.  All other droplets are unaffected.
//
// Usage:
//   1. Call setTarget(dropletId, worldPoint) when the user grabs a droplet.
//   2. Call updateGrabPoint(worldPoint) every frame while dragging.
//   3. Call clearTarget() on mouse-release.
//
// The force is applied in ExternalForceOperator::apply(), which calls
// IForceField::sample() per-vertex — but sample() has no droplet-ID
// context.  We therefore take a different approach: the SimulationSystem
// calls applyToDroplet() directly if it finds this field in the composite,
// OR we keep a "current droplet id" set just before each droplet step.
//
// The simplest integration point given your existing architecture is to
// store the grabbed-droplet id in App and temporarily activate/deactivate
// this field around each droplet's solver step (see integration notes in
// droplet_drag_interactor.h).

#include "wd/forces/iforce_field.h"

namespace wd {

class DropletDragForceField final : public IForceField {
public:
    // Activate drag toward worldGrab for the given droplet id.
    void setTarget(int dropletId, const Vec3& worldGrab) {
        active_      = true;
        targetId_    = dropletId;
        grabPoint_   = worldGrab;
    }

    void updateGrabPoint(const Vec3& worldGrab) { grabPoint_ = worldGrab; }

    void clearTarget() {
        active_    = false;
        targetId_  = -1;
    }

    bool active()    const { return active_; }
    int  targetId()  const { return targetId_; }

    // Called by ExternalForceOperator for every vertex of the *current* droplet.
    // App sets activeDropletId_ to the droplet being stepped so we can gate.
    void setActiveDropletId(int id) { activeDropletId_ = id; }

    Vec3 sample(const Vec3& vertexPos, double /*timeSec*/) const override {
        if (!active_ || activeDropletId_ != targetId_) return Vec3::Zero();
        Vec3 toGrab = grabPoint_ - vertexPos;
        double dist = toGrab.norm();
        if (dist < 1e-8) return Vec3::Zero();
        // Linear spring: strength falls off so it doesn't explode at close range.
        double force = strength_ * std::min(dist, maxDist_);
        return force * toGrab / dist;
    }

    void setStrength(double s) { strength_ = s; }
    void setMaxDist(double d)  { maxDist_  = d; }

private:
    bool   active_          = false;
    int    targetId_        = -1;
    int    activeDropletId_ = -1;
    Vec3   grabPoint_       = Vec3::Zero();
    double strength_        = 40.0;   // tune to taste
    double maxDist_         = 0.5;
};

} // namespace wd