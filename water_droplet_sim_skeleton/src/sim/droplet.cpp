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

    // 3. Shape Descriptors (based on the contact boundary loop)
    const auto& boundary = tpl_->boundaryLoop();
    if (boundary.size() >= 3) {
        Vec3 center = Vec3::Zero();
        for (int idx : boundary) {
            center += X_.row(idx).transpose();
        }
        center /= static_cast<double>(boundary.size());

        // Covariance matrix of boundary vertices
        Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
        double maxR = 0.0;
        for (int idx : boundary) {
            Vec3 d = X_.row(idx).transpose() - center;
            // Zero out Y component if you strictly want a 2D footprint analysis
            // d.y() = 0; 
            C += d * d.transpose();
            maxR = std::max(maxR, d.norm());
        }
        C /= static_cast<double>(boundary.size());
        
        derived_.footprintRadius = maxR;

        // Eigendecomposition to find principal axes of the footprint
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        if (es.info() == Eigen::Success) {
            auto vals = es.eigenvalues();
            auto vecs = es.eigenvectors();
            
            // vals(2) is the largest variance, vals(0) is the smallest
            double largest = std::max(vals(2), 1e-12);
            double smallest = std::max(vals(0), 1e-12);
            
            derived_.elongationRatio = std::sqrt(largest / smallest);
            derived_.principalAxis = vecs.col(2).normalized();
        }
    } else {
        // Fallbacks for point/line topologies
        derived_.footprintRadius = 0.0;
        derived_.elongationRatio = 1.0;
        derived_.principalAxis = Vec3::UnitX();
    }
}

} // namespace wd