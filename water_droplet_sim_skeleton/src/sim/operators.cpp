#include "wd/sim/operators.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_set>

namespace wd {

class ExternalForceOperator {
public:
    void apply(SimState& state, const Vec3& gravity, double density, Eigen::MatrixXd& a_internal) {
        // Gravity is applied directly to total acceleration later, 
        // but we can initialize the base acceleration matrix here.
        for (int i = 0; i < state.x.rows(); ++i) {
            a_internal.row(i) += gravity.transpose();
        }
    }
};

class CurvatureFlowOperator {
public:
    void apply(const SimState& state, double gamma, double density, Eigen::MatrixXd& a_internal) {
        for (int i = 0; i < state.x.rows(); ++i) {
            Vec3 delta_x = Vec3::Zero();
            double sum_w = 0.0;
            
            auto it_i = state.w.find(i);
            if (it_i != state.w.end()) {
                for (int j : state.neighbours[i]) {
                    double w_ij = 1.0;
                    auto it_j = it_i->second.find(j);
                    if (it_j != it_i->second.end()) {
                        w_ij = it_j->second;
                    }
                    Vec3 xj = state.x.row(j).transpose();
                    Vec3 xi = state.x.row(i).transpose();
                    delta_x += (xj - xi) * w_ij;
                    sum_w += w_ij;
                }
                if (sum_w > 0) delta_x /= sum_w;
            }
            
            Vec3 f_st = delta_x * gamma;
            a_internal.row(i) += (f_st / density).transpose();
        }
    }
};

class ViscosityOperator {
public:
    // Calculates delta_v and applies energy-aware damping
    void apply(SimState& state, double dt, double mu, double eta, double damping_gain) {
        Eigen::MatrixXd delta_v = Eigen::MatrixXd::Zero(state.v.rows(), 3);
        
        // 1. Compute Velocity Laplacian
        for (int i = 0; i < state.x.rows(); ++i) {
            Vec3 dv = Vec3::Zero();
            double sum_w = 0.0;
            auto it_i = state.w.find(i);
            if (it_i != state.w.end()) {
                for (int j : state.neighbours[i]) {
                    double w_ij = 1.0;
                    auto it_j = it_i->second.find(j);
                    if (it_j != it_i->second.end()) w_ij = it_j->second;
                    
                    Vec3 vj = state.v.row(j).transpose();
                    Vec3 vi = state.v.row(i).transpose();
                    dv += (vj - vi) * w_ij;
                    sum_w += w_ij;
                }
                if (sum_w > 0) delta_v.row(i) = (dv / sum_w).transpose();
            }
        }

        // 2. Apply Damping
        double mean_speed = 0.0;
        for (int i = 0; i < state.v.rows(); ++i) {
            Vec3 vi = state.v.row(i).transpose();
            Vec3 dvi = delta_v.row(i).transpose();
            
            Vec3 v_damped = vi * (1.0 - mu * dt) + dvi * (eta * dt);
            double speed = vi.norm();
            
            double lin = std::max(0.0, mu) * damping_gain;
            double quad = (0.2 + 0.3 * std::max(0.0, eta)) * damping_gain;
            double damping = 1.0 / (1.0 + lin * dt + quad * speed * dt);
            
            state.v.row(i) = (v_damped * damping).transpose();
            mean_speed += speed;
        }

        // 3. Global rest stabilization
        mean_speed /= state.x.rows();
        if (mean_speed < 0.25) {
            double factor = std::max(0.0, 1.0 - 0.3 * dt * damping_gain);
            state.v *= factor;
        }
    }
};

class VolumeCorrector {
public:
    void apply(const SimState& state, double k_v, double density, Eigen::MatrixXd& a_internal) {
        for (int i = 0; i < state.x.rows(); ++i) {
            Vec3 ni = state.normals.row(i).transpose();
            Vec3 f_vol = ni * (k_v * (state.rest_volume - state.volume));
            a_internal.row(i) += (f_vol / density).transpose();
        }
    }
};

class ContactLineOperator {
public:
    void apply(const SimState& state, double boundary_alpha, double receding_angle, 
               double advancing_angle, double adhesion_dist, double density, Eigen::MatrixXd& a_internal) {
        
        Vec3 ground_normal(0, 0, 1);

        for (int i = 0; i < state.x.rows(); ++i) {
            if (state.x(i, 2) > adhesion_dist) continue; // contact_mask logic

            Vec3 n_li = state.normals.row(i).transpose();
            double dot_val = std::max(-1.0, std::min(1.0, n_li.dot(ground_normal)));
            double angle = std::acos(dot_val);

            Vec3 n_p = n_li - ground_normal * dot_val;
            Vec3 n_p_dir = n_p.normalized();

            if (n_p_dir.norm() < 1e-10) continue;
            if (angle > receding_angle && angle < advancing_angle) continue;

            Vec3 f_boundary = Vec3::Zero();
            if (angle <= receding_angle) {
                f_boundary = n_p_dir * (boundary_alpha * (angle - receding_angle));
            } else {
                f_boundary = n_p_dir * (boundary_alpha * (angle - advancing_angle));
            }

            a_internal.row(i) += (f_boundary / density).transpose();
        }
    }
};

class CollisionProjector {
public:
    void apply(SimState& state, double dt, double friction_coeff, double adhesion_dist) {
        for (int i = 0; i < state.x.rows(); ++i) {
            if (state.x(i, 2) <= adhesion_dist) {
                // Ground collision
                if (state.x(i, 2) < 0) {
                    state.x(i, 2) = 0;
                    if (state.v(i, 2) < 0) {
                        state.v(i, 2) *= -0.2; // Bounce damping
                    }
                }

                // Friction
                Vec3 v_horiz(state.v(i, 0), state.v(i, 1), 0);
                double speed = v_horiz.norm();
                if (speed > 0) {
                    double drop = friction_coeff * dt;
                    if (speed < drop) {
                        state.v(i, 0) = 0;
                        state.v(i, 1) = 0;
                    } else {
                        double scale = (speed - drop) / speed;
                        state.v(i, 0) *= scale;
                        state.v(i, 1) *= scale;
                    }
                }

                // Surface slip drag
                state.v(i, 0) *= (1.0 - 0.05 * dt);
                state.v(i, 1) *= (1.0 - 0.05 * dt);
            }
        }
    }
};

} // namespace wd