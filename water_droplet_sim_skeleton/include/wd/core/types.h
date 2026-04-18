#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace wd {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using MatX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using MatX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using SparseMat = Eigen::SparseMatrix<double>;

struct Ray {
    Vec3 origin = Vec3::Zero();
    Vec3 dir = Vec3::UnitZ();
};

struct AABB {
    Vec3 min = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 max = Vec3::Constant(-std::numeric_limits<double>::infinity());
};

struct SurfaceSample {
    Vec3 position = Vec3::Zero();
    Vec3 normal = Vec3::UnitY();
    Vec3 tangentU = Vec3::UnitX();
    Vec3 tangentV = Vec3::UnitZ();
    double signedDistance = 0.0;
};

struct PickHit {
    bool hit = false;
    Vec3 position = Vec3::Zero();
    Vec3 normal = Vec3::UnitY();
    double t = 0.0;
    int dropletId = -1;
};

struct MaterialParams {
    double density = 1.0;

    double surfaceTension = 1.0;
    double friction = 0.15;
    double viscousDamping = 0.10;
    double laplacianViscosity = 0.02;

    double advContactAngleDeg = 95.0;
    double recContactAngleDeg = 70.0;
    double contactStiffness = 0.5;

    double volumeStiffness = 0.8;
};

struct SolverParams {
    double dt = 1.0 / 120.0;
    int substeps = 1;

    bool enableExternalForce = true;
    bool enableCollision = true;
    bool enableViscosity = true;
    bool enableCurvatureFlow = true;
    bool enableContactAngle = true;
    bool enableVolumeCorrect = true;

    double collisionPushoutEps = 1e-4;
};

struct RenderParams {
    double ior = 1.33;
    double fresnelBias = 0.02;
    double fresnelScale = 0.98;
    double fresnelPower = 5.0;

    double refractionScale = 0.03;
    double specularPower = 64.0;
    bool enableThickness = true;
};

struct SimulationStats {
    double simMs = 0.0;
    double meanVolumeError = 0.0;
    double maxVolumeError = 0.0;
    int dropletCount = 0;
};

struct RenderStats {
    double renderMs = 0.0;
    int frameWidth = 0;
    int frameHeight = 0;
};

} // namespace wd
