#include "wd/sim/operators.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_set>

namespace wd {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::unordered_set<int> makeBoundarySet(const Droplet& d) {
    return {d.boundaryLoop().begin(), d.boundaryLoop().end()};
}

Vec3 tangentComponent(const Vec3& v, const Vec3& n) {
    return v - v.dot(n) * n;
}

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

std::vector<std::vector<int>> buildAdjacency(const Droplet& d) {
    std::vector<std::vector<int>> adj(static_cast<size_t>(d.positions().rows()));
    auto add = [&](int a, int b) {
        auto& list = adj[static_cast<size_t>(a)];
        if (std::find(list.begin(), list.end(), b) == list.end()) list.push_back(b);
    };

    for (int i = 0; i < d.faces().rows(); ++i) {
        int a = d.faces()(i, 0), b = d.faces()(i, 1), c = d.faces()(i, 2);
        add(a, b); add(a, c);
        add(b, a); add(b, c);
        add(c, a); add(c, b);
    }
    return adj;
}

std::vector<std::map<int, double>> computeCotangentWeights(const Droplet& d) {
    std::vector<std::map<int, double>> w(static_cast<size_t>(d.positions().rows()));
    const auto& X = d.positions();

    for (int f = 0; f < d.faces().rows(); ++f) {
        int i0 = d.faces()(f, 0);
        int i1 = d.faces()(f, 1);
        int i2 = d.faces()(f, 2);

        Vec3 x0 = X.row(i0).transpose();
        Vec3 x1 = X.row(i1).transpose();
        Vec3 x2 = X.row(i2).transpose();

        Vec3 cross0 = (x1 - x0).cross(x2 - x0);
        Vec3 cross1 = (x2 - x1).cross(x0 - x1);
        Vec3 cross2 = (x0 - x2).cross(x1 - x2);

        double norm0 = std::max(cross0.norm(), 1e-5);
        double norm1 = std::max(cross1.norm(), 1e-5);
        double norm2 = std::max(cross2.norm(), 1e-5);

        double cot0 = (x1 - x0).dot(x2 - x0) / norm0;
        double cot1 = (x2 - x1).dot(x0 - x1) / norm1;
        double cot2 = (x0 - x2).dot(x1 - x2) / norm2;

        double w0 = std::max(0.0, cot2 / 2.0);
        double w1 = std::max(0.0, cot0 / 2.0);
        double w2 = std::max(0.0, cot1 / 2.0);

        w[static_cast<size_t>(i0)][i1] += w0;
        w[static_cast<size_t>(i1)][i0] += w0;
        w[static_cast<size_t>(i1)][i2] += w1;
        w[static_cast<size_t>(i2)][i1] += w1;
        w[static_cast<size_t>(i2)][i0] += w2;
        w[static_cast<size_t>(i0)][i2] += w2;
    }

    return w;
}

struct LaplacianContext {
    std::vector<std::vector<int>> adjacency;
    std::vector<std::map<int, double>> weights;
};

LaplacianContext buildLaplacianContext(const Droplet& d) {
    return {buildAdjacency(d), computeCotangentWeights(d)};
}

MatX3d computeLaplacian(const MatX3d& values, const std::vector<std::vector<int>>& adj,
                        const std::vector<std::map<int, double>>& w) {
    MatX3d delta = MatX3d::Zero(values.rows(), 3);

    for (int i = 0; i < values.rows(); ++i) {
        double sumW = 0.0;
        Vec3 vi = values.row(i).transpose();
        for (int j : adj[static_cast<size_t>(i)]) {
            auto it = w[static_cast<size_t>(i)].find(j);
            double wij = (it != w[static_cast<size_t>(i)].end()) ? it->second : 1.0;
            Vec3 vj = values.row(j).transpose();
            delta.row(i) += (wij * (vj - vi)).transpose();
            sumW += wij;
        }

        if (sumW > 0.0) {
            delta.row(i) /= sumW;
        }
    }

    return delta;
}

double signedTetVolume(const Vec3& a, const Vec3& b, const Vec3& c) {
    return a.dot(b.cross(c)) / 6.0;
}

} // namespace

void ExternalForceOperator::apply(
    Droplet& drop, const ISurface&, const IForceField& field, double timeSec, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        Vec3 a = field.sample(x, timeSec);
        U.row(i) += (dt * a).transpose();
    }
}

void CollisionProjector::apply(
    Droplet& drop, const ISurface& surface, double pushoutEps, double, double dt) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    const SurfaceMaterialParams& surfaceMaterial = surface.material();
    double adhesionDist = std::max(0.0, surfaceMaterial.adhesionDistance);
    double frictionCoeff = std::max(0.0, surfaceMaterial.friction);
    double horizDamping = std::max(0.0, 1.0 - 0.05 * dt);

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        SurfaceSample s = surface.closestSample(x);
        if (s.signedDistance <= adhesionDist) {
            Vec3 v = U.row(i).transpose();

            // Collision response only when penetrating the surface.
            if (s.signedDistance < 0.0) {
                X.row(i) = (s.position + pushoutEps * s.normal).transpose();
                Vec3 relative = v - surfaceMaterial.solidVelocity;
                double vn = relative.dot(s.normal);
                if (vn < 0.0) {
                    relative -= vn * s.normal;
                    v = surfaceMaterial.solidVelocity + relative;
                }
            }

            // Sliding friction / surface adhesion in the tangent plane.
            Vec3 relative = v - surfaceMaterial.solidVelocity;
            Vec3 vTan = tangentComponent(relative, s.normal);
            double speed = vTan.norm();
            if (speed > 0.0) {
                double dropSpeed = frictionCoeff * dt;
                if (speed < dropSpeed) {
                    relative -= vTan;
                } else {
                    relative -= (dropSpeed / speed) * vTan;
                }
            }

            // Simple linear damping for tangential velocity.
            vTan = tangentComponent(relative, s.normal);
            relative = relative.dot(s.normal) * s.normal + horizDamping * vTan;
            v = surfaceMaterial.solidVelocity + relative;
            U.row(i) = v.transpose();
        }
    }
}

void ViscosityOperator::apply(Droplet& drop, double dt) const {
    auto& U = drop.velocities();
    double damping = std::max(0.0, 1.0 - drop.material().viscousDamping * dt);
    U *= damping;

    double eta = drop.material().laplacianViscosity;
    if (eta <= 0.0) return;

    auto ctx = buildLaplacianContext(drop);
    MatX3d deltaV = computeLaplacian(U, ctx.adjacency, ctx.weights);
    U += eta * dt * deltaV;
}

void CurvatureFlowOperator::apply(Droplet& drop, const ISurface& surface, double dt) const {
    auto& U = drop.velocities();
    const auto& X = drop.positions();

    auto ctx = buildLaplacianContext(drop);
    MatX3d deltaX = computeLaplacian(X, ctx.adjacency, ctx.weights);

    double gamma = drop.material().surfaceTension;
    double kv = drop.material().volumeStiffness;
    if (gamma <= 0.0 && kv <= 0.0) return;

    MatX3d fSt = gamma * deltaX;

    VolumeCorrector vc;
    double currentVolume = vc.computeClosedVolume(drop, surface);
    double volumeDelta = drop.targetVolume() - currentVolume;
    MatX3d normals = drop.derived().vertexNormals;
    MatX3d fVol = kv * volumeDelta * normals;

    double density = std::max(drop.material().density, 1e-8);
    U += (dt / density) * (fSt + fVol);
}

void ContactLineOperator::apply(Droplet& drop, const ISurface& surface, double dt, double) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();
    const auto& normals = drop.derived().vertexNormals;
    const SurfaceMaterialParams& surfaceMaterial = surface.material();
    double adhesionDist = std::max(0.0, surfaceMaterial.adhesionDistance);

    double alpha = std::max(0.0, surfaceMaterial.contactStiffness);
    if (alpha <= 0.0) return;

    double receding = degToRad(surfaceMaterial.recContactAngleDeg);
    double advancing = degToRad(surfaceMaterial.advContactAngleDeg);
    if (receding > advancing) std::swap(receding, advancing);

    double density = std::max(drop.material().density, 1e-8);

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        SurfaceSample s = surface.closestSample(x);
        if (s.signedDistance > adhesionDist) continue;

        Vec3 nI = s.normal.normalized();
        Vec3 nL = normals.row(i).transpose();
        double dotVal = std::clamp(nL.dot(nI), -1.0, 1.0);
        double angle = std::acos(dotVal);

        Vec3 nP = nL - dotVal * nI;
        double nPNorm = nP.norm();
        if (nPNorm <= 1e-12) continue;

        Vec3 nPDir = nP / nPNorm;
        if (receding < angle && angle < advancing) continue;
        double delta = (angle <= receding) ? (angle - receding) : (angle - advancing);

        Vec3 fBoundary = alpha * delta * nPDir;
        U.row(i) += ((dt / density) * fBoundary).transpose();
    }
}

double VolumeCorrector::computeClosedVolume(const Droplet& drop, const ISurface& surface) const {
    double vol = 0.0;

    for (int i = 0; i < drop.faces().rows(); ++i) {
        Vec3 a = drop.positions().row(drop.faces()(i, 0)).transpose();
        Vec3 b = drop.positions().row(drop.faces()(i, 1)).transpose();
        Vec3 c = drop.positions().row(drop.faces()(i, 2)).transpose();
        vol += signedTetVolume(a, b, c);
    }

    if (drop.boundaryLoop().size() >= 3) {
        Vec3 center = Vec3::Zero();
        for (int idx : drop.boundaryLoop()) {
            center += surface.projectPoint(drop.positions().row(idx).transpose());
        }
        center /= static_cast<double>(drop.boundaryLoop().size());

        for (size_t k = 0; k < drop.boundaryLoop().size(); ++k) {
            int ia = drop.boundaryLoop()[k];
            int ib = drop.boundaryLoop()[(k + 1) % drop.boundaryLoop().size()];
            Vec3 a = surface.projectPoint(drop.positions().row(ia).transpose());
            Vec3 b = surface.projectPoint(drop.positions().row(ib).transpose());
            vol += signedTetVolume(center, b, a);
        }
    }

    return std::abs(vol);
}

void VolumeCorrector::apply(Droplet& drop, const ISurface& surface, double) const {
    double current = computeClosedVolume(drop, surface);
    if (current <= 1e-12 || drop.targetVolume() <= 1e-12) return;

    double ratio = drop.targetVolume() / current;
    double scale = std::clamp(1.0 + drop.material().volumeStiffness * (std::cbrt(ratio) - 1.0), 0.5, 1.5);

    auto& X = drop.positions();
    auto boundary = makeBoundarySet(drop);

    for (int i = 0; i < X.rows(); ++i) {
        if (boundary.count(i)) continue;
        Vec3 x = X.row(i).transpose();
        SurfaceSample s = surface.closestSample(x);
        Vec3 offset = x - s.position;
        X.row(i) = (s.position + scale * offset).transpose();
    }
}

} // namespace wd
