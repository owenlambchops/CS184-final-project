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
    derived_.avgVelocity = Vec3::Zero();
    if (U_.rows() > 0) {
        derived_.avgVelocity = U_.colwise().mean().transpose();
    }

}

} // namespace wd
