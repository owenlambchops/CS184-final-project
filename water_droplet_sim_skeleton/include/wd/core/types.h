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
    // Tuned defaults: allow capillary lift without overdamping lower vertices.
    double surfaceTension = 2000.0;        // gamma
    double viscousDamping = 5.0;           // mu
    double laplacianViscosity = 8.0;       // eta
    double volumeStiffness = 2000.0;       // k_v
    double density = 1.0;
    bool enableLocalVolumeCorrection = false;
    bool enableGlobalVolumeCorrection = false;
};

struct SurfaceMaterialParams {
    Vec3 solidVelocity = Vec3::Zero();
    double friction = 0.05;

    double advContactAngleDeg = 90.0;
    double recContactAngleDeg = 30.0;
    double contactStiffness = 0.5;
};

struct SurfaceRenderParams {
    double ior = 1.52;
    double opacity = 0.32;
    Vec3 tintColor = Vec3(0.78, 0.90, 1.0);
};

struct SurfaceRenderMesh {
    MatX3d positions;
    MatX3d normals;
    MatX3i faces;
};

struct SolverParams {
    double dt = 1.0 / 120.0;
    int substeps = 1;

    bool enableExternalForce = true;
    bool enableCollision = true;
    bool enableViscosity = true;
    bool enableCurvatureFlow = true;
    bool enableContactAngle = false;
    bool enableVolumeCorrect = true;
    bool enableVertexRepulsion = false;
    
    double collisionPushoutEps = 1e-4;
    double adhesionDistance = 0.005;
    double maxVelocity = 15.0;
    double maxInternalAccel = 200.0;
    double vertexDamping = 0.0;
    int contactBandProjIterations = 3;
    double contactBandProjRelaxation = 0.2;
    double contactBandProjTargetRatio = 0.9;
};

enum class RenderDebugView : int {
    Final = 0,
    SceneColor = 1,
    EnvironmentMap = 2,
    SceneDepth = 3,
    DropletDepth = 4,
    DropletNormal = 5,
    Thickness = 6,
    Wireframe = 7,
    Caustics = 8,
};

struct RenderParams {
    double ior = 1.33;
    double fresnelBias = 0.02;
    double fresnelScale = 0.98;
    double fresnelPower = 5.0;

    double refractionScale = 0.03;
    double maxThickness = 1.0;
    double debugDepthRange = 8.0;
    double absorptionStrength = 1.35;
    Vec3 absorptionColor = Vec3(0.65, 0.24, 0.10);
    bool enableThickness = true;
    bool enableCaustics = true;
    double causticStrength = 0.45;
    double causticPointSize = 2.0;
    RenderDebugView debugView = RenderDebugView::Final;
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
