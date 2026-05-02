#pragma once

#include "wd/render/refractive_renderer.h" 
#include "wd/core/scene.h"
#include "wd/core/types.h"
#include "wd/forces/droplet_drag_force_field.h"
#include "wd/interaction/input_router.h"
#include "wd/interaction/picker.h"

#include <memory>
#include <cmath>
#include <utility>

namespace wd {

class DropletDragInteractor {
public:
    explicit DropletDragInteractor(std::shared_ptr<DropletDragForceField> field)
        : field_(field) {}

    void update(const InputState& input,
                const Camera&     camera,
                int               viewportW,
                int               viewportH,
                const Scene&      scene) {
        if (!field_) return;

        // ── Release ──────────────────────────────────────────────────────────
        if (input.leftReleased) {
            field_->clearTarget();
            dragging_ = false;
            return;
        }

        // ── Press: pick a droplet ────────────────────────────────────────────
        if (input.leftPressed) {
            Ray ray = picker_.makeCameraRay(
                input.mouseX, input.mouseY, viewportW, viewportH, camera);

            int    bestId  = -1;
            double bestT   = 1e30;
            Vec3   bestHit = Vec3::Zero();

            for (const auto& d : scene.droplets()) {
                Vec3   center = d->derived().centerOfMass;
                double radius = d->derived().footprintRadius * 1.5; // generous sphere
                double t      = raySphereIntersect(ray, center, radius);
                if (t > 0.0 && t < bestT) {
                    bestT   = t;
                    bestId  = d->id();
                    bestHit = ray.origin + t * ray.dir;
                }
            }

            if (bestId >= 0) {
                dragging_      = true;
                dragPlaneOrig_ = bestHit;
                dragPlaneNorm_ = (Vec3::Zero() - camera.position()).normalized();
                field_->setTarget(bestId, bestHit);
                return;
            }
            // miss → don't start drag
        }

        // ── Held: slide grab point along drag plane ──────────────────────────
        if (dragging_ && (input.leftDown)) {
            Ray ray = picker_.makeCameraRay(
                input.mouseX, input.mouseY, viewportW, viewportH, camera);

            double denom = ray.dir.dot(dragPlaneNorm_);
            if (std::abs(denom) > 1e-8) {
                double t = (dragPlaneOrig_ - ray.origin).dot(dragPlaneNorm_) / denom;
                if (t > 0.0) {
                    field_->updateGrabPoint(ray.origin + t * ray.dir);
                }
            }
        }
    }

private:
    // Returns t of first intersection with sphere(center, radius), or -1.
    static double raySphereIntersect(const Ray& ray, const Vec3& center, double radius) {
        Vec3   oc = ray.origin - center;
        double b  = oc.dot(ray.dir);
        double c  = oc.dot(oc) - radius * radius;
        double disc = b * b - c;
        if (disc < 0.0) return -1.0;
        double t = -b - std::sqrt(disc);
        return t > 0.0 ? t : -1.0;
    }

    Picker                                    picker_;
    std::shared_ptr<DropletDragForceField>    field_;
    bool                                      dragging_      = false;
    Vec3                                      dragPlaneOrig_ = Vec3::Zero();
    Vec3                                      dragPlaneNorm_ = Vec3::UnitZ();
};

} // namespace wd