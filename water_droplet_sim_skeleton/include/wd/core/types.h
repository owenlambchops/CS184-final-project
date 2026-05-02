#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <map>
#include <cmath>


namespace wd {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using MatX3d = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using MatX3i = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>;
using SparseMat = Eigen::SparseMatrix<double>;
using DenseMat = Eigen::MatrixXd;
using Weights = std::map<int, std::map<int, double>>;

// Stores adjacency data needed for the Loop Subdivision stencil
struct Edge {
    int v1 = -1, v2 = -1;
    int opposite1 = -1, opposite2 = -1;
    int new_vid = -1; // The index of the newly inserted vertex
};

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
    double surfaceTension = 4000.0;           // gamma
    double viscousDamping = 8.0;          // mu
    double laplacianViscosity = 16.0;      // eta
    double density = 1.0;
    double contactStiffness = 3.0;         // alpha
    double friction = 0.2;
    // damping_gain=1.6, # Apply new robust damping
    double advContactAngleDeg = 89.0;      // theta_adv
    double recContactAngleDeg = 91.0;      // theta_rec    
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
    double adhesionDistance = 0.05;
    double maxVelocity = 15.0;
    double maxInternalAccel = 200.0;
    double vertexDamping = 0.0;
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

// Helper to extract geometry data from the Eigen matrices
struct SimState {
    MatX3d x; // Positions
    MatX3d v; // Velocities
    MatX3d a; // Accelerations
    MatX3i faces;
    std::vector<std::vector<int>> neighbours;
    Weights w;
    MatX3d normals;
    double volume;
    double rest_volume;
};

} // namespace wd
