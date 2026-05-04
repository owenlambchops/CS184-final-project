#pragma once
#include "wd/core/types.h"

namespace wd {

class IForceField {
public:
    virtual ~IForceField() = default;
    virtual Vec3 sample(const Vec3& worldPoint, double timeSec) const = 0;
};

class ConstantForceField final : public IForceField {
public:
    explicit ConstantForceField(const Vec3& force);
    void setForce(const Vec3& force);
    const Vec3& force() const;
    Vec3 sample(const Vec3& worldPoint, double timeSec) const override;

private:
    Vec3 force_;
};

class DragForceField final : public IForceField {
public:
    void setActive(bool active);
    bool active() const;

    void setCenter(const Vec3& center);
    const Vec3& center() const;

    void setRadius(double radius);
    void setStrength(double strength);
    void setDirection(const Vec3& direction);

    Vec3 sample(const Vec3& worldPoint, double timeSec) const override;

private:
    bool active_ = false;
    Vec3 center_ = Vec3::Zero();
    Vec3 direction_ = Vec3::Zero();
    double radius_ = 0.25;
    double strength_ = 1.0;
};

class CompositeForceField final : public IForceField {
public:
    void addField(std::shared_ptr<IForceField> field);
    void clear();
    Vec3 sample(const Vec3& worldPoint, double timeSec) const override;

private:
    std::vector<std::shared_ptr<IForceField>> fields_;
};

} // namespace wd
