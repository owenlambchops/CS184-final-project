#pragma once
#include "wd/core/types.h"

namespace wd {

struct DropletDerivedData {
    MatX3d vertexNormals;
    Eigen::VectorXd vertexMeanEdgeLength;
    Eigen::VectorXd vertexSpeed;
    Eigen::VectorXd vertexCurvature;
    Vec3 centerOfMass = Vec3::Zero();
    Vec3 avgVelocity = Vec3::Zero();
    double currentVolume = 0.0;
    double restVolume = 0.0;
    double minEdgeLength = 0.0;
    double meanEdgeLength = 0.0;
    double maxEdgeLength = 0.0;
    double restMeanEdgeLength = 0.0;
    double footprintRadius = 0.0;
    double minorAxisRadius = 0.0;
    double elongationRatio = 1.0;
    Vec3 principalAxis = Vec3::UnitX();
};

} // namespace wd
