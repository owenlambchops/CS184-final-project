#include "wd/sim/operators.h"
#include <algorithm>
#include <cmath>
#include <map>

namespace wd {
namespace {

Vec3 tangentComponent(const Vec3& v, const Vec3& n) {
    return v - v.dot(n) * n;
}

std::vector<std::vector<int>> buildAdjacency(const Droplet& d) {
    std::vector<std::vector<int>> adj(static_cast<size_t>(d.positions().rows()));
    auto add = [&](int a, int b) {
        auto& list = adj[static_cast<size_t>(a)];
        if (std::find(list.begin(), list.end(), b) == list.end()) list.push_back(b);
    };

    for (int i = 0; i < d.faces().rows(); ++i) {
        int a = d.faces()(i, 0), b = d.faces()(i, 1), c = d.faces()(i, 2);
        add(a, b);
        add(a, c);
        add(b, a);
        add(b, c);
        add(c, a);
        add(c, b);
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
    Droplet& drop, const ISurface& surface, double pushoutEps, double adhesionDist, double dt) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    double frictionCoeff = std::max(0.0, drop.material().friction);
    double horizDamping = std::max(0.0, 1.0 - 0.05 * dt);

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        SurfaceSample s = surface.closestSample(x);
        if (s.signedDistance <= adhesionDist) {
            Vec3 v = U.row(i).transpose();

            // Collision and bounce only when penetrating the surface.
            if (s.signedDistance < 0.0) {
                X.row(i) = (s.position + pushoutEps * s.normal).transpose();
                double vn = v.dot(s.normal);
                if (vn < 0.0) {
                    v -= 1.2 * vn * s.normal;
                }
            }

            // Sliding friction / surface adhesion in the tangent plane.
            Vec3 vTan = tangentComponent(v, s.normal);
            double speed = vTan.norm();
            if (speed > 0.0) {
                double dropSpeed = frictionCoeff * dt;
                if (speed < dropSpeed) {
                    v -= vTan;
                } else {
                    v -= (dropSpeed / speed) * vTan;
                }
            }

            // Simple linear damping for tangential velocity.
            vTan = tangentComponent(v, s.normal);
            v = v.dot(s.normal) * s.normal + horizDamping * vTan;
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

void ContactLineOperator::apply(Droplet&, const ISurface&, double, double) const {
}

double VolumeCorrector::computeClosedVolume(const Droplet& drop, const ISurface&) const {
    double vol = 0.0;

    for (int i = 0; i < drop.faces().rows(); ++i) {
        Vec3 a = drop.positions().row(drop.faces()(i, 0)).transpose();
        Vec3 b = drop.positions().row(drop.faces()(i, 1)).transpose();
        Vec3 c = drop.positions().row(drop.faces()(i, 2)).transpose();
        vol += signedTetVolume(a, b, c);
    }

    return std::abs(vol);
}

void VolumeCorrector::apply(Droplet& drop, const ISurface& surface, double) const {
    double current = computeClosedVolume(drop, surface);
    if (current <= 1e-12 || drop.targetVolume() <= 1e-12) return;

    double ratio = drop.targetVolume() / current;
    double scale = std::clamp(1.0 + drop.material().volumeStiffness * (std::cbrt(ratio) - 1.0), 0.5, 1.5);

    auto& X = drop.positions();
    Vec3 center = X.colwise().mean().transpose();

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        Vec3 offset = x - center;
        X.row(i) = (center + scale * offset).transpose();
    }
}

} // namespace wd
