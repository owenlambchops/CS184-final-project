#include "wd/sim/merge_split_controller.h"
#include <utility>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace wd {
namespace { // Anonymous namespace for internal helper functions

// Möller–Trumbore ray-triangle intersection algorithm
bool rayTriangleIntersect(const Vec3& orig, const Vec3& dir,
                          const Vec3& v0, const Vec3& v1, const Vec3& v2,
                          double& t) {
    const double EPSILON = 1e-8;
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    Vec3 h = dir.cross(edge2);
    double a = edge1.dot(h);
    
    // If a is near zero, ray is parallel to the triangle
    if (a > -EPSILON && a < EPSILON) return false; 
    
    double f = 1.0 / a;
    Vec3 s = orig - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    
    Vec3 q = s.cross(edge1);
    double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;
    
    t = f * edge2.dot(q);
    return t > EPSILON; // Ray intersection must be strictly in the forward direction
}

// Computes the closed volume of an arbitrary triangle mesh
double computeMeshVolume(const MatX3d& V, const MatX3i& F) {
    if (V.rows() == 0 || F.rows() == 0) return 0.0;
    Vec3 com = V.colwise().mean().transpose();
    double vol = 0.0;
    for (int i = 0; i < F.rows(); ++i) {
        Vec3 v0 = V.row(F(i, 0)).transpose() - com;
        Vec3 v1 = V.row(F(i, 1)).transpose() - com;
        Vec3 v2 = V.row(F(i, 2)).transpose() - com;
        vol += v0.dot(v1.cross(v2)) / 6.0;
    }
    return std::abs(vol);
}

} // namespace

MergeSplitController::MergeSplitController(DropletFactory factory) : factory_(std::move(factory)) {}

void MergeSplitController::process(std::vector<std::unique_ptr<Droplet>>& droplets, const ISurface& surface) {
    if (enableMerge) {
        for (size_t i = 0; i < droplets.size(); ++i) {
            for (size_t j = i + 1; j < droplets.size(); ++j) {
                if (shouldMerge(*droplets[i], *droplets[j])) {
                    
                    const Droplet& dA = *droplets[i];
                    const Droplet& dB = *droplets[j];

                    // 1. Calculate explicit new volume to preserve (V_new = V_1 + V_2)
                    double vA = dA.derived().currentVolume;
                    double vB = dB.derived().currentVolume;
                    double vNew = vA + vB;
                    
                    // 2. Spawn a clean template droplet to act as our shrinkwrap base.
                    // This gives us a mathematically perfect, non-intersecting spherical 2-manifold.
                    auto merged = factory_.spawnMerged(nextId_++, dA, dB, surface, postMergeDamping);
                    
                    MatX3d V = merged->positions();
                    const MatX3i& F = merged->faces();
                    Vec3 com = merged->derived().centerOfMass;
                    
                    // 3. Ray-Cast Shrinkwrap: Project the clean vertices onto the outer hull of A & B.
                    // By finding the *maximum* intersection distance, we naturally delete internal intersecting triangles 
                    // and seamlessly fuse the outer crossing boundaries together.
                    for (int k = 0; k < V.rows(); ++k) {
                        Vec3 orig = com;
                        Vec3 dir = (V.row(k).transpose() - com).normalized();
                        double max_t = 0.0;
                        bool hit = false;
                        
                        // Check intersections against Droplet A
                        for(int fi = 0; fi < dA.faces().rows(); ++fi) {
                            Vec3 v0 = dA.positions().row(dA.faces()(fi, 0)).transpose();
                            Vec3 v1 = dA.positions().row(dA.faces()(fi, 1)).transpose();
                            Vec3 v2 = dA.positions().row(dA.faces()(fi, 2)).transpose();
                            double t;
                            if(rayTriangleIntersect(orig, dir, v0, v1, v2, t)) {
                                if(t > max_t) { max_t = t; hit = true; }
                            }
                        }
                        
                        // Check intersections against Droplet B
                        for(int fi = 0; fi < dB.faces().rows(); ++fi) {
                            Vec3 v0 = dB.positions().row(dB.faces()(fi, 0)).transpose();
                            Vec3 v1 = dB.positions().row(dB.faces()(fi, 1)).transpose();
                            Vec3 v2 = dB.positions().row(dB.faces()(fi, 2)).transpose();
                            double t;
                            if(rayTriangleIntersect(orig, dir, v0, v1, v2, t)) {
                                if(t > max_t) { max_t = t; hit = true; }
                            }
                        }
                        
                        if (hit) {
                            V.row(k) = (orig + dir * max_t).transpose();
                        } else {
                            // Fallback: If ray misses due to extreme concavity, snap to the nearest known vertex
                            double min_dist = 1e9;
                            Vec3 expected_p = V.row(k).transpose();
                            Vec3 best_p = expected_p;
                            
                            for(int vi = 0; vi < dA.positions().rows(); ++vi) {
                                double d = (dA.positions().row(vi).transpose() - expected_p).norm();
                                if(d < min_dist) { min_dist = d; best_p = dA.positions().row(vi).transpose(); }
                            }
                            for(int vi = 0; vi < dB.positions().rows(); ++vi) {
                                double d = (dB.positions().row(vi).transpose() - expected_p).norm();
                                if(d < min_dist) { min_dist = d; best_p = dB.positions().row(vi).transpose(); }
                            }
                            V.row(k) = best_p.transpose();
                        }
                    }
                    
                    // 4. Force strict Volume Preservation
                    double current_V = computeMeshVolume(V, F);
                    double scale = std::cbrt(std::max(vNew, 1e-8) / std::max(current_V, 1e-8));
                    for (int k = 0; k < V.rows(); ++k) {
                        Vec3 p = V.row(k).transpose();
                        V.row(k) = (com + (p - com) * scale).transpose(); // Scale positions outward from COM
                    }
                    
                    // 5. Apply the final geometry
                    merged->positions() = V;
                    
                    // Distribute blended velocity
                    Vec3 initialVelocity = Vec3::Zero();
                    if (merged->velocities().rows() > 0) initialVelocity = merged->velocities().row(0).transpose();
                    MatX3d newVelocities = MatX3d::Zero(V.rows(), 3);
                    for(int k = 0; k < newVelocities.rows(); ++k) {
                        newVelocities.row(k) = initialVelocity.transpose();
                    }
                    merged->velocities() = newVelocities;

                    // Re-calculate normals and mass
                    merged->updateDerived();
                    merged->setTargetVolume(vNew); // Update target to precisely V_1 + V_2
                    
                    // 6. Clean up old droplets
                    droplets.erase(droplets.begin() + static_cast<long>(j));
                    droplets.erase(droplets.begin() + static_cast<long>(i));
                    droplets.push_back(std::move(merged));
                    
                    return; // Process one merge at a time to keep topological updates stable
                }
            }
        }
    }

    if (enableSplit) {
        for (size_t i = 0; i < droplets.size(); ++i) {
            if (shouldSplit(*droplets[i])) {
                int firstId = nextId_++;
                int secondId = nextId_++;
                auto children = factory_.spawnSplit(firstId, secondId, *droplets[i], surface, 0.5);
                droplets.erase(droplets.begin() + static_cast<long>(i));
                droplets.push_back(std::move(children.first));
                droplets.push_back(std::move(children.second));
                return;
            }
        }
    }
}

// Debugged collision detection using 3D Axis-Aligned Bounding Boxes (AABB)
bool MergeSplitController::shouldMerge(const Droplet& a, const Droplet& b) const {
    const MatX3d& Va = a.positions();
    const MatX3d& Vb = b.positions();
    if (Va.rows() == 0 || Vb.rows() == 0) return false;

    // Calculate AABB for both droplets
    Vec3 minA = Va.colwise().minCoeff();
    Vec3 maxA = Va.colwise().maxCoeff();
    Vec3 minB = Vb.colwise().minCoeff();
    Vec3 maxB = Vb.colwise().maxCoeff();

    // Expand bounding boxes by a small threshold to catch them right before intersection
    double eps = mergeDistanceFactor * std::min(a.derived().meanEdgeLength, b.derived().meanEdgeLength);
    minA.array() -= eps; maxA.array() += eps;
    minB.array() -= eps; maxB.array() += eps;

    // Check for 3D overlap
    bool overlapX = (minA.x() <= maxB.x() && maxA.x() >= minB.x());
    bool overlapY = (minA.y() <= maxB.y() && maxA.y() >= minB.y());
    bool overlapZ = (minA.z() <= maxB.z() && maxA.z() >= minB.z());

    return overlapX && overlapY && overlapZ;
}

bool MergeSplitController::shouldSplit(const Droplet& d) const {
    return d.targetVolume() >= splitMinVolume && d.derived().elongationRatio > splitElongationThreshold;
}

} // namespace wd