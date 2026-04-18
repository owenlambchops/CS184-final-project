#include "wd/sim/operators.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace wd {
namespace {

std::unordered_set<int> makeBoundarySet(const Droplet& d) {
    return {d.boundaryLoop().begin(), d.boundaryLoop().end()};
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

double signedTetVolume(const Vec3& a, const Vec3& b, const Vec3& c) {
    return a.dot(b.cross(c)) / 6.0;
}

} // namespace

void ExternalForceOperator::apply(
    Droplet& drop, const ISurface& surface, const IForceField& field, double timeSec, double dt) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    double invDensity = 1.0 / std::max(drop.material().density, 1e-8);

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        Vec3 f = field.sample(x, timeSec);
        Vec3 n = surface.normalAt(x);
        Vec3 tangentF = f - f.dot(n) * n;
        U.row(i) += (dt * invDensity * tangentF).transpose();
    }
}

void CollisionProjector::apply(Droplet& drop, const ISurface& surface, double pushoutEps) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();

    for (int idx : drop.boundaryLoop()) {
        Vec3 x = X.row(idx).transpose();
        X.row(idx) = surface.projectPoint(x).transpose();

        Vec3 n = surface.normalAt(x);
        Vec3 v = U.row(idx).transpose();
        U.row(idx) = (v - v.dot(n) * n).transpose();
    }

    for (int i = 0; i < X.rows(); ++i) {
        Vec3 x = X.row(i).transpose();
        SurfaceSample s = surface.closestSample(x);
        if (s.signedDistance < pushoutEps) {
            X.row(i) = (s.position + pushoutEps * s.normal).transpose();

            Vec3 v = U.row(i).transpose();
            double vn = v.dot(s.normal);
            if (vn < 0.0) v -= vn * s.normal;
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

    auto adj = buildAdjacency(drop);
    MatX3d oldU = U;

    for (int i = 0; i < U.rows(); ++i) {
        if (adj[static_cast<size_t>(i)].empty()) continue;
        Vec3 avg = Vec3::Zero();
        for (int j : adj[static_cast<size_t>(i)]) avg += oldU.row(j).transpose();
        avg /= static_cast<double>(adj[static_cast<size_t>(i)].size());

        Vec3 ui = oldU.row(i).transpose();
        U.row(i) = (ui + eta * dt * (avg - ui)).transpose();
    }
}

void CurvatureFlowOperator::apply(Droplet& drop, double dt) const {
    auto& X = drop.positions();
    auto boundary = makeBoundarySet(drop);
    auto adj = buildAdjacency(drop);
    MatX3d oldX = X;

    double strength = drop.material().surfaceTension;
    if (strength <= 0.0) return;

    for (int i = 0; i < X.rows(); ++i) {
        if (boundary.count(i)) continue;
        if (adj[static_cast<size_t>(i)].empty()) continue;

        Vec3 avg = Vec3::Zero();
        for (int j : adj[static_cast<size_t>(i)]) avg += oldX.row(j).transpose();
        avg /= static_cast<double>(adj[static_cast<size_t>(i)].size());

        Vec3 xi = oldX.row(i).transpose();
        X.row(i) = (xi + strength * dt * (avg - xi)).transpose();
    }
}

void ContactLineOperator::apply(Droplet& drop, const ISurface& surface, double) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    double friction = std::clamp(drop.material().friction, 0.0, 1.0);

    for (int idx : drop.boundaryLoop()) {
        Vec3 x = X.row(idx).transpose();
        SurfaceSample s = surface.closestSample(x);
        X.row(idx) = s.position.transpose();

        Vec3 v = U.row(idx).transpose();
        v = v - v.dot(s.normal) * s.normal;
        U.row(idx) = ((1.0 - friction) * v).transpose();
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
