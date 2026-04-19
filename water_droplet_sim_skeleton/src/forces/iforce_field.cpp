#include "wd/forces/iforce_field.h"
#include <algorithm>

namespace wd {

ConstantForceField::ConstantForceField(const Vec3& force) : force_(force) {}
void ConstantForceField::setForce(const Vec3& force) { force_ = force; }
const Vec3& ConstantForceField::force() const { return force_; }
Vec3 ConstantForceField::sample(const Vec3&, double) const { return force_; }

void DragForceField::setActive(bool active) { active_ = active; }
bool DragForceField::active() const { return active_; }
void DragForceField::setCenter(const Vec3& center) { center_ = center; }
const Vec3& DragForceField::center() const { return center_; }
void DragForceField::setRadius(double radius) { radius_ = std::max(1e-6, radius); }
void DragForceField::setStrength(double strength) { strength_ = strength; }
void DragForceField::setDirection(const Vec3& direction) { direction_ = direction; }

Vec3 DragForceField::sample(const Vec3& x, double) const {
    if (!active_) return Vec3::Zero();

    Vec3 toCenter = center_ - x;
    double dist = toCenter.norm();
    if (dist > radius_) return Vec3::Zero();

    double falloff = 1.0 - dist / radius_;
    Vec3 dir = Vec3::Zero();
    if (direction_.norm() > 1e-8) {
        dir = direction_.normalized();
    } else if (dist > 1e-8) {
        dir = toCenter / dist;
    }
    return strength_ * falloff * falloff * dir;
}

void CompositeForceField::addField(std::shared_ptr<IForceField> field) {
    if (field) fields_.push_back(std::move(field));
}

void CompositeForceField::clear() { fields_.clear(); }

Vec3 CompositeForceField::sample(const Vec3& worldPoint, double timeSec) const {
    Vec3 sum = Vec3::Zero();
    for (const auto& f : fields_) {
        if (f) sum += f->sample(worldPoint, timeSec);
    }
    return sum;
}

} // namespace wd
