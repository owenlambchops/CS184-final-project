#include "wd/sim/droplet.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace wd {

Droplet::Droplet(int id, std::shared_ptr<const DropletTemplate> tpl, const MaterialParams& material)
    : id_(id), tpl_(std::move(tpl)), material_(material) {
    
    // Initialize primary state from the template
    X_ = tpl_->restVertices();
    U_ = MatX3d::Zero(X_.rows(), 3);
    
    // Initialize derived data for the first time
    updateDerived();
    
    // Set the initial volume as the rest volume target
    derived_.restVolume = derived_.currentVolume;
}

void Droplet::updateDerived() {
    int n = static_cast<int>(X_.rows());
    
    // 1. Update Vertex Normals (Area-weighted)
    derived_.vertexNormals = MatX3d::Zero(n, 3);
    const auto& F = tpl_->faces(); // Fetch topology from template

    for (int i = 0; i < F.rows(); ++i) {
        int a = F(i, 0), b = F(i, 1), c = F(i, 2);
        Vec3 va = X_.row(a).transpose();
        Vec3 vb = X_.row(b).transpose();
        Vec3 vc = X_.row(c).transpose();
        
        // Area-weighted normal of the face
        Vec3 fn = (vb - va).cross(vc - va);
        if (fn.norm() > 1e-12) fn.normalize();

        // Accumulate to vertices
        derived_.vertexNormals.row(a) += fn.transpose();
        derived_.vertexNormals.row(b) += fn.transpose();
        derived_.vertexNormals.row(c) += fn.transpose();
    }

    // Normalize accumulated vertex normals
    for (int i = 0; i < n; ++i) {
        Vec3 normal = derived_.vertexNormals.row(i).transpose();
        if (normal.norm() < 1e-12) normal = Vec3::UnitY(); // Fallback
        derived_.vertexNormals.row(i) = normal.normalized().transpose();
    }

    // 2. Kinematics
    derived_.centerOfMass = X_.colwise().mean().transpose();
    derived_.avgVelocity = Vec3::Zero();
    if (U_.rows() > 0) {
        derived_.avgVelocity = U_.colwise().mean().transpose();
    }

    // 3. Shape Descriptors from all vertices (ground-plane footprint)
    if (n >= 3) {
        Vec3 center = Vec3::Zero();
        for (int i = 0; i < n; ++i) {
            center += X_.row(i).transpose();
        }
        center /= static_cast<double>(n);

        MatX3d C = MatX3d::Zero(3, 3);
        double maxR = 0.0;
        for (int i = 0; i < n; ++i) {
            Vec3 d = X_.row(i).transpose() - center;
            // Footprint is measured on the ground plane (x/y), so ignore vertical spread.
            d.z() = 0.0;
            C += d * d.transpose();
            maxR = std::max(maxR, d.norm());
        }
        C /= static_cast<double>(n);

        derived_.footprintRadius = maxR;

        // Principal footprint direction = largest-variance eigenvector in x/y.
        Eigen::SelfAdjointEigenSolver<MatX3d> es(C);
        if (es.info() == Eigen::Success) {
            const auto vals = es.eigenvalues();
            Vec3 axis = es.eigenvectors().col(2).normalized();
            axis.z() = 0.0;
            if (axis.norm() > 1e-12) {
                axis.normalize();
            } else {
                axis = Vec3::UnitX();
            }

            double largest = std::max(vals(2), 1e-12);
            double smallest = std::max(vals(1), 1e-12);
            derived_.elongationRatio = std::sqrt(largest / smallest);
            derived_.principalAxis = axis;
        } else {
            derived_.elongationRatio = 1.0;
            derived_.principalAxis = Vec3::UnitX();
        }
    } else {
        // Fallbacks for point/line topologies
        derived_.footprintRadius = 0.0;
        derived_.elongationRatio = 1.0;
        derived_.principalAxis = Vec3::UnitX();
    }
}

} // namespace wd
