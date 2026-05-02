#include "wd/sim/droplet.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

namespace wd {

Droplet::Droplet(int id, std::shared_ptr<const DropletTemplate> tpl, const MaterialParams& material)
    : id_(id), tpl_(std::move(tpl)), material_(material) {
    X_ = tpl_->restVertices();
    U_ = MatX3d::Zero(X_.rows(), 3);
    F_ = tpl_->faces();
    E_ = tpl_->edges();
    boundaryLoop_ = tpl_->boundaryLoop();
    updateDerived();
    derived_.restVolume = derived_.currentVolume;
    derived_.restMeanEdgeLength = derived_.meanEdgeLength;
}

void Droplet::updateDerived() {
    // TODO(remesh-step2): After topology edits, call rebuildEdgesFromFaces() before updateDerived().
    // TODO(remesh-step3): Add validation hooks:
    // - detect non-manifold edges
    // - detect flipped/degenerate faces
    // - compute min triangle area for stability diagnostics

    const int n = static_cast<int>(X_.rows());
    if (n == 0) return;

    // 1) Normals + closed volume.
    const Vec3 com = X_.colwise().mean().transpose();
    double volumeSigned = 0.0;
    derived_.vertexNormals = MatX3d::Zero(n, 3);
    const auto& F = F_;

    for (int i = 0; i < F.rows(); ++i) {
        const int a = F(i, 0), b = F(i, 1), c = F(i, 2);
        const Vec3 va = X_.row(a).transpose();
        const Vec3 vb = X_.row(b).transpose();
        const Vec3 vc = X_.row(c).transpose();

        Vec3 fn = (vb - va).cross(vc - va);
        if (fn.dot(va - com) < 0.0) fn = -fn;

        volumeSigned += (va - com).dot((vb - com).cross(vc - com)) / 6.0;

        derived_.vertexNormals.row(a) += fn.transpose();
        derived_.vertexNormals.row(b) += fn.transpose();
        derived_.vertexNormals.row(c) += fn.transpose();
    }
    derived_.currentVolume = std::abs(volumeSigned);

    for (int i = 0; i < n; ++i) {
        Vec3 normal = derived_.vertexNormals.row(i).transpose();
        if (normal.norm() < 1e-12) normal = Vec3::UnitY();
        derived_.vertexNormals.row(i) = normal.normalized().transpose();
    }

    // 2) Kinematics.
    derived_.centerOfMass = com;
    derived_.avgVelocity = Vec3::Zero();
    if (U_.rows() > 0) derived_.avgVelocity = U_.colwise().mean().transpose();

    derived_.vertexSpeed = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) derived_.vertexSpeed(i) = U_.row(i).norm();

    // 3) Edge statistics and per-vertex mean incident edge length.
    derived_.vertexMeanEdgeLength = Eigen::VectorXd::Zero(n);
    std::vector<int> edgeDegree(n, 0);
    // Fallback: if runtime edges are missing/stale, rebuild from faces.
    if (E_.empty()) {
        std::set<std::pair<int, int>> edgeSet;
        auto addEdge = [&](int a, int b) {
            if (a > b) std::swap(a, b);
            edgeSet.emplace(a, b);
        };
        for (int fi = 0; fi < F.rows(); ++fi) {
            addEdge(F(fi, 0), F(fi, 1));
            addEdge(F(fi, 1), F(fi, 2));
            addEdge(F(fi, 2), F(fi, 0));
        }
        E_.clear();
        E_.reserve(edgeSet.size());
        for (const auto& e : edgeSet) E_.emplace_back(e.first, e.second);
    }
    const auto& E = E_;

    double minLen = std::numeric_limits<double>::infinity();
    double maxLen = 0.0;
    double sumLen = 0.0;
    int edgeCount = 0;

    for (const auto& e : E) {
        const int i = e.x();
        const int j = e.y();
        if (i < 0 || j < 0 || i >= n || j >= n) continue;

        const double len = (X_.row(j).transpose() - X_.row(i).transpose()).norm();
        minLen = std::min(minLen, len);
        maxLen = std::max(maxLen, len);
        sumLen += len;
        ++edgeCount;

        derived_.vertexMeanEdgeLength(i) += len;
        derived_.vertexMeanEdgeLength(j) += len;
        edgeDegree[i] += 1;
        edgeDegree[j] += 1;
    }

    for (int i = 0; i < n; ++i) {
        if (edgeDegree[i] > 0) derived_.vertexMeanEdgeLength(i) /= static_cast<double>(edgeDegree[i]);
    }

    if (edgeCount > 0) {
        derived_.minEdgeLength = minLen;
        derived_.meanEdgeLength = sumLen / static_cast<double>(edgeCount);
        derived_.maxEdgeLength = maxLen;
    } else {
        derived_.minEdgeLength = 0.0;
        derived_.meanEdgeLength = 0.0;
        derived_.maxEdgeLength = 0.0;
    }

    // 4) Curvature proxy from uniform Laplacian magnitude.
    std::vector<std::set<int>> adj(n);
    for (int fi = 0; fi < F.rows(); ++fi) {
        const int a = F(fi, 0), b = F(fi, 1), c = F(fi, 2);
        adj[a].insert(b); adj[a].insert(c);
        adj[b].insert(a); adj[b].insert(c);
        adj[c].insert(a); adj[c].insert(b);
    }

    derived_.vertexCurvature = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
        if (adj[i].empty()) continue;
        Vec3 avg = Vec3::Zero();
        for (int j : adj[i]) avg += X_.row(j).transpose();
        avg /= static_cast<double>(adj[i].size());
        derived_.vertexCurvature(i) = (avg - X_.row(i).transpose()).norm();
    }

    // 5) Shape descriptors.
    if (n >= 3) {
        Vec3 center = X_.colwise().mean().transpose();
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        double maxR = 0.0;
        for (int i = 0; i < n; ++i) {
            Vec3 d = X_.row(i).transpose() - center;
            d.z() = 0.0;
            C += d * d.transpose();
            maxR = std::max(maxR, d.norm());
        }
        C /= static_cast<double>(n);
        derived_.footprintRadius = maxR;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        if (es.info() == Eigen::Success) {
            const auto vals = es.eigenvalues();
            Vec3 axis = es.eigenvectors().col(2).normalized();
            axis.z() = 0.0;
            if (axis.norm() > 1e-12) axis.normalize();
            else axis = Vec3::UnitX();

            const double largest = std::max(vals(2), 1e-12);
            const double smallest = std::max(vals(1), 1e-12);
            derived_.elongationRatio = std::sqrt(largest / smallest);
            derived_.principalAxis = axis;
        } else {
            derived_.elongationRatio = 1.0;
            derived_.principalAxis = Vec3::UnitX();
        }
    } else {
        derived_.footprintRadius = 0.0;
        derived_.elongationRatio = 1.0;
        derived_.principalAxis = Vec3::UnitX();
    }
}

} // namespace wd
