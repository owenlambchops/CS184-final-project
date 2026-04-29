#include <vector>
#include <cmath>
#include <algorithm>
#include <map>

// --- libMesh Headers ---
#include "libmesh/libmesh.h"
#include "libmesh/mesh.h"
#include "libmesh/mesh_generation.h"
#include "libmesh/elem.h"
#include "libmesh/node.h"

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/viewer.h"
#include "CGL/renderer.h"
#include "CGL/misc.h"

typedef std::vector<int> Face;
typedef std::map<int, std::map<int, double>> Weights;

// Container for our extracted simulation arrays
struct SimState {
    std::vector<Vector3D> x;
    std::vector<Vector3D> v;
    std::vector<Vector3D> a;
    std::vector<Face> faces;
    std::vector<std::vector<int>> neighbours;
    double V_0; // Initial volume
};



// Equivalent to compute_cotangent_weights()
Weights compute_cotangent_weights(const std::vector<Vector3D>& x, const std::vector<Face>& faces) {
    Weights w;
    for (const auto& f : faces) {
        int i0 = f[0], i1 = f[1], i2 = f[2];
        Vector3D v0 = x[i0], v1 = x[i1], v2 = x[i2];

        Vector3D e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
        
        // Cotangents via dot / norm of cross
        auto get_cot = [](Vector3D a, Vector3D b) {
            Vector3D c = a.cross(b);
            double n = c.norm();
            if (n < 1e-10) return 0.0;
            return a.dot(b) / n;
        };

        double cot0 = get_cot(v1 - v0, v2 - v0);
        double cot1 = get_cot(v2 - v1, v0 - v1);
        double cot2 = get_cot(v0 - v2, v1 - v2);

        double w0 = std::max(0.0, cot2 / 2.0);
        double w1 = std::max(0.0, cot0 / 2.0);
        double w2 = std::max(0.0, cot1 / 2.0);

        w[i0][i1] += w0; w[i1][i0] += w0;
        w[i1][i2] += w1; w[i2][i1] += w1;
        w[i2][i0] += w2; w[i0][i2] += w2;
    }
    return w;
}

// Equivalent to compute_vertex_normals_and_volume()
std::pair<std::vector<Vector3D>, double> compute_vertex_normals_and_volume(const std::vector<Vector3D>& x, const std::vector<Face>& faces) {
    std::vector<Vector3D> normals(x.size(), Vector3D(0, 0, 0));
    double V = 0.0;
    Vector3D com(0, 0, 0);
    for (const auto& v : x) { com += v; }
    com /= (double)x.size();

    for (const auto& f : faces) {
        int i0 = f[0], i1 = f[1], i2 = f[2];
        Vector3D x0 = x[i0], x1 = x[i1], x2 = x[i2];
        Vector3D n_face = (x1 - x0).cross(x2 - x0);

        if (n_face.dot(x0 - com) < 0) {
            n_face = n_face * -1.0;
        }

        normals[i0] += n_face;
        normals[i1] += n_face;
        normals[i2] += n_face;

        V += (x0 - com).dot((x1 - com).cross(x2 - com)) / 6.0;
    }

    for (auto& n : normals) {
        n = n.normalized();
    }
    return {normals, std::abs(V)};
}

// Equivalent to compute_laplacians()
void compute_laplacians(const std::vector<Vector3D>& x, const std::vector<Vector3D>& v, 
                        const std::vector<std::vector<int>>& neighbours, const Weights& w,
                        std::vector<Vector3D>& delta_x, std::vector<Vector3D>& delta_v) {
    delta_x.assign(x.size(), Vector3D(0, 0, 0));
    delta_v.assign(v.size(), Vector3D(0, 0, 0));

    for (int i = 0; i < (int)x.size(); ++i) {
        double sum_w = 0.0;
        auto it_i = w.find(i);
        if (it_i == w.end()) continue;

        for (int j : neighbours[i]) {
            double w_ij = 1.0;
            auto it_j = it_i->second.find(j);
            if (it_j != it_i->second.end()) w_ij = it_j->second;

            delta_x[i] += (x[j] - x[i]) * w_ij;
            delta_v[i] += (v[j] - v[i]) * w_ij;
            sum_w += w_ij;
        }
        if (sum_w > 0) {
            delta_x[i] /= sum_w;
            delta_v[i] /= sum_w;
        }
    }
}

// Equivalent to apply_surface_interactions()
void apply_surface_interactions(std::vector<Vector3D>& x, std::vector<Vector3D>& v, 
                                double dt, double friction_coeff, double adhesion_dist) {
    for (int i = 0; i < (int)x.size(); ++i) {
        if (x[i].z <= adhesion_dist) {
            if (x[i].z < 0) {
                x[i].z = 0;
                if (v[i].z < 0) v[i].z *= -0.2;
            }

            Vector3D v_horiz(v[i].x, v[i].y, 0);
            double speed = v_horiz.norm();
            if (speed > 0) {
                double drop = friction_coeff * dt;
                if (speed < drop) {
                    v[i].x = 0; v[i].y = 0;
                } else {
                    double scale = (speed - drop) / speed;
                    v[i].x *= scale;
                    v[i].y *= scale;
                }
            }
            // Surface slip drag
            v[i].x *= (1.0 - 0.05 * dt);
            v[i].y *= (1.0 - 0.05 * dt);
        }
    }
}

// Equivalent to boundary_force()
std::vector<Vector3D> boundary_force(double alpha, const std::vector<Vector3D>& n_l, 
                                const std::vector<bool>& contact_mask, 
                                double receding_angle, double advancing_angle) {
    std::vector<Vector3D> f_boundary(n_l.size(), Vector3D(0, 0, 0));
    Vector3D ground_normal(0, 0, 1);

    for (int i = 0; i < (int)n_l.size(); ++i) {
        if (!contact_mask[i]) continue;

        Vector3D n_li = n_l[i];
        double dot_val = std::max(-1.0, std::min(1.0, n_li.dot(ground_normal)));
        double angle = std::acos(dot_val);

        Vector3D n_p = n_li - ground_normal * dot_val;
        Vector3D n_p_dir = n_p.normalized();

        if (n_p_dir.norm() < 1e-10) continue;

        if (angle > receding_angle && angle < advancing_angle) continue;

        if (angle <= receding_angle) {
            f_boundary[i] = n_p_dir * (alpha * (angle - receding_angle));
        } else {
            f_boundary[i] = n_p_dir * (alpha * (angle - advancing_angle));
        }
    }
    return f_boundary;
}

// Equivalent to compute_accelerations()
std::pair<std::vector<Vector3D>, std::vector<Vector3D>> compute_accelerations(
    const std::vector<Vector3D>& x, const std::vector<Vector3D>& v, const std::vector<Face>& faces,
    const std::vector<std::vector<int>>& neighbours, double V_0, Vector3D g, double gamma,
    double k_v, double boundary_alpha, double receding_angle, double advancing_angle,
    double adhesion_dist, double max_internal_accel, double density) {

    Weights w = compute_cotangent_weights(x, faces);
    auto [n, V] = compute_vertex_normals_and_volume(x, faces);
    
    std::vector<Vector3D> delta_x, delta_v;
    compute_laplacians(x, v, neighbours, w, delta_x, delta_v);

    std::vector<bool> contact_mask(x.size());
    for (int i = 0; i < (int)x.size(); ++i) contact_mask[i] = (x[i].z <= adhesion_dist);

    std::vector<Vector3D> f_boundary = boundary_force(boundary_alpha, n, contact_mask, receding_angle, advancing_angle);

    std::vector<Vector3D> a_total(x.size());
    for (int i = 0; i < (int)x.size(); ++i) {
        Vector3D f_st = delta_x[i] * gamma;
        Vector3D f_vol = n[i] * (k_v * (V_0 - V));
        Vector3D a_internal = (f_st + f_vol + f_boundary[i]) / density;

        double a_int_norm = a_internal.norm();
        double scale = std::min(1.0, max_internal_accel / std::max(a_int_norm, 1e-8));
        
        a_total[i] = g + (a_internal * scale);
    }

    return {a_total, delta_v};
}

// Equivalent to update_forward_euler()
void update_forward_euler(std::vector<Vector3D>& a, std::vector<Vector3D>& v, std::vector<Vector3D>& x,
                        const std::vector<Face>& faces, const std::vector<std::vector<int>>& neighbours,
                        double V_0, double dt, Vector3D g, double gamma, double mu, double eta, double k_v,
                        double friction_coeff, double boundary_alpha, double receding_angle, double advancing_angle,
                        double adhesion_dist, double max_internal_accel, double density, double damping_gain) {
    
    auto [a_total, delta_v] = compute_accelerations(x, v, faces, neighbours, V_0, g, gamma, k_v, 
                                                    boundary_alpha, receding_angle, advancing_angle, 
                                                    adhesion_dist, max_internal_accel, density);
    
    double mean_speed = 0;
    for (int i = 0; i < (int)x.size(); ++i) {
        // Base damping
        Vector3D v_damped = v[i] * (1.0 - mu * dt) + delta_v[i] * (eta * dt);

        // Energy-aware damping
        double speed = v[i].norm();
        double lin = std::max(0.0, mu) * damping_gain;
        double quad = (0.2 + 0.3 * std::max(0.0, eta)) * damping_gain;
        double damping = 1.0 / (1.0 + lin * dt + quad * speed * dt);
        
        v_damped *= damping;
        mean_speed += speed;
        
        // Integration
        v[i] = v_damped + a_total[i] * dt;
        // Clip
        auto clip = [](double val, double limit) { return std::max(-limit, std::min(limit, val)); };
        v[i].x = clip(v[i].x, 15.0); v[i].y = clip(v[i].y, 15.0); v[i].z = clip(v[i].z, 15.0);

        x[i] += v[i] * dt;
        a[i] = a_total[i];
    }

    mean_speed /= x.size();
    if (mean_speed < 0.25) {
        double factor = std::max(0.0, 1.0 - 0.3 * dt * damping_gain);
        for (auto& vel : v) vel *= factor;
    }

    apply_surface_interactions(x, v, dt, friction_coeff, adhesion_dist);
}

// Logic matches Python's update_velocity_verlet()
void update_velocity_verlet(std::vector<Vector3D>& a, std::vector<Vector3D>& v, std::vector<Vector3D>& x,
                        const std::vector<Face>& faces, const std::vector<std::vector<int>>& neighbours,
                        double V_0, double dt, Vector3D g, double gamma, double mu, double eta, double k_v,
                        double friction_coeff, double boundary_alpha, double receding_angle, double advancing_angle,
                        double adhesion_dist, double max_internal_accel, double density, double damping_gain) {
    
    std::vector<Vector3D> x_new(x.size());
    for (int i = 0; i < (int)x.size(); ++i) {
        x_new[i] = x[i] + v[i] * dt + a[i] * (dt * dt * 0.5);
    }

    auto [a_new, delta_v] = compute_accelerations(x_new, v, faces, neighbours, V_0, g, gamma, k_v, 
                                                  boundary_alpha, receding_angle, advancing_angle, 
                                                  adhesion_dist, max_internal_accel, density);

    double mean_speed = 0;
    for (int i = 0; i < (int)x.size(); ++i) {
        Vector3D v_new = v[i] + (a[i] + a_new[i]) * (dt * 0.5);
        Vector3D v_damped = v_new * (1.0 - mu * dt) + delta_v[i] * (eta * dt);

        double speed = v_new.norm();
        double lin = std::max(0.0, mu) * damping_gain;
        double quad = (0.2 + 0.3 * std::max(0.0, eta)) * damping_gain;
        double damping = 1.0 / (1.0 + lin * dt + quad * speed * dt);
        
        v_damped *= damping;
        mean_speed += speed;

        auto clip = [](double val, double limit) { return std::max(-limit, std::min(limit, val)); };
        v[i].x = clip(v_damped.x, 15.0); v[i].y = clip(v_damped.y, 15.0); v[i].z = clip(v_damped.z, 15.0);
        x[i] = x_new[i];
        a[i] = a_new[i];
    }

    mean_speed /= x.size();
    if (mean_speed < 0.25) {
        double factor = std::max(0.0, 1.0 - 0.3 * dt * damping_gain);
        for (auto& vel : v) vel *= factor;
    }

    apply_surface_interactions(x, v, dt, friction_coeff, adhesion_dist);
}


/**
 * Safely generates a libMesh sphere and extracts it into contiguous arrays
 * suited for real-time physics iteration.
 */
SimState generate_physics_mesh(libMesh::Mesh& mesh, double radius, int segments) {
    SimState state;
    state.V_0 = 4.0/3.0 * M_PI * std::pow(radius, 3);

    // 1. Ask libMesh to build the geometry using 3-node triangles
    libMesh::MeshTools::Generation::build_sphere(mesh, radius, segments, libMesh::TRI3);
    mesh.prepare_for_use(); // Crucial: Finalizes element connectivity and nodal assignments

    // 2. Refinement: Map libMesh IDs to 0-based contiguous array indices.
    // libMesh node IDs are NOT guaranteed to be 0,1,2,3 in parallel environments.
    std::map<libMesh::dof_id_type, int> node_id_map;
    int current_idx = 0;
    
    for (const auto& node : mesh.node_ptr_range()) {
        node_id_map[node->id()] = current_idx++;
        state.x.push_back(Vector3D((*node)(0), (*node)(1), (*node)(2)));
        state.v.push_back(Vector3D(0, 0, 0));
        state.a.push_back(Vector3D(0, 0, 0));
    }

    // 3. Extract Element (Face) data safely using the map
    std::vector<std::set<int>> neighbor_sets(state.x.size());
    
    for (const auto& elem : mesh.element_ptr_range()) {
        if (elem->type() == libMesh::TRI3) {
            // Safely map libMesh node IDs to our contiguous vectors
            int i0 = node_id_map[elem->node_id(0)];
            int i1 = node_id_map[elem->node_id(1)];
            int i2 = node_id_map[elem->node_id(2)];

            state.faces.push_back({i0, i1, i2});

            // Build unique topological connections for laplacian calculations
            neighbor_sets[i0].insert(i1); neighbor_sets[i0].insert(i2);
            neighbor_sets[i1].insert(i0); neighbor_sets[i1].insert(i2);
            neighbor_sets[i2].insert(i0); neighbor_sets[i2].insert(i1);
        }
    }

    // 4. Flatten sets into optimized vector arrays for cache locality during simulation
    state.neighbours.resize(state.x.size());
    for (int i = 0; i < (int)state.x.size(); ++i) {
        state.neighbours[i].assign(neighbor_sets[i].begin(), neighbor_sets[i].end());
    }

    return state;
}