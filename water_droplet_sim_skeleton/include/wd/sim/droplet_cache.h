#pragma once
#include "wd/core/types.h"

namespace wd {

struct DropletDerivedData {
    MatX3d vertexNormals;
    Vec3 centerOfMass = Vec3::Zero();
    Vec3 avgVelocity = Vec3::Zero();

    double currentVolume = 0.0;
    double footprintRadius = 0.0;
    double elongationRatio = 1.0;

    Vec3 principalAxis = Vec3::UnitX();
};

} // namespace wd
