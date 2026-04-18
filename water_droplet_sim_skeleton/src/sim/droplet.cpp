#include "wd/sim/droplet.h"
#include <Eigen/Eigenvalues>
#include <algorithm>

namespace wd {

Droplet::Droplet(int id, std::shared_ptr<const DropletTemplate> tpl, const MaterialParams& material)
    : id_(id), tpl_(std::move(tpl)), material_(material) {
    X_ = tpl_->restVertices();
    U_ = MatX3d::Zero(X_.rows(), 3);
    updateDerived();
}

void Droplet::updateDerived() {
    int n = static_cast<int>(X_.rows());
    derived_.vertexNormals = MatX3d::Zero(n, 3);

    for (int i = 0; i < faces().rows(); ++i) {
        int a = faces()(i, 0), b = faces()(i, 1), c = faces()(i, 2);
        Vec3 va = X_.row(a).transpose();
        Vec3 vb = X_.row(b).transpose();
        Vec3 vc = X_.row(c).transpose();
        Vec3 fn = (vb - va).cross(vc - va);
        if (fn.norm() > 1e-12) fn.normalize();

        derived_.vertexNormals.row(a) += fn.transpose();
        derived_.vertexNormals.row(b) += fn.transpose();
        derived_.vertexNormals.row(c) += fn.transpose();
    }

    for (int i = 0; i < n; ++i) {
        Vec3 normal = derived_.vertexNormals.row(i).transpose();
        if (normal.norm() < 1e-12) normal = Vec3::UnitY();
        derived_.vertexNormals.row(i) = normal.normalized().transpose();
    }

    derived_.centerOfMass = X_.colwise().mean().transpose();
    derived_.avgVelocity = U_.rows() > 0 ? U_.colwise().mean().transpose() : Vec3::Zero();

    if (boundaryLoop().size() >= 3) {
        Vec3 center = Vec3::Zero();
        for (int idx : boundaryLoop()) center += X_.row(idx).transpose();
        center /= static_cast<double>(boundaryLoop().size());

        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        double maxR = 0.0;
        for (int idx : boundaryLoop()) {
            Vec3 d = X_.row(idx).transpose() - center;
            C += d * d.transpose();
            maxR = std::max(maxR, d.norm());
        }

        C /= static_cast<double>(boundaryLoop().size());
        derived_.footprintRadius = maxR;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        if (es.info() == Eigen::Success) {
            auto vals = es.eigenvalues();
            auto vecs = es.eigenvectors();
            double largest = std::max(vals(2), 1e-12);
            double smallest = std::max(vals(0), 1e-12);
            derived_.elongationRatio = std::sqrt(largest / smallest);
            derived_.principalAxis = vecs.col(2).normalized();
        }
    }
}

} // namespace wd
