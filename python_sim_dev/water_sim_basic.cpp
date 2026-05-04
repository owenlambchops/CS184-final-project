// Real-Time Interactive Water Droplets (Euler & Advanced Damping)
// Simulates a deformable water droplet on a surface using Vertex Dynamics.

#include <pybind11/pybind11.h>
#include <vector>
#include <array>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Surface_mesh.h>
#include <unordered_map>
#include <cassert>
#include <string>
#include <limits>
#include <optional>
#include <utility>

// Represents a 3D vector/point
struct Vec3 {
    float x, y, z;

    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s)        const { return {x * s,   y * s,   z * s  }; }
    Vec3 operator/(float s)        const { return {x / s,   y / s,   z / s  }; }

    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float norm()             const { return std::sqrt(dot(*this)); }
};

// Represents a triangular face (indices into vertex array)
using Face = std::array<int, 3>;

struct ConvexHullResult {
    std::vector<Face> simplices;
};

struct DropletMesh {
    std::vector<Vec3>               vertices;
    std::vector<Face>               faces;
    std::map<int, std::vector<int>> neighbours;
};

// Normalize a single Vec3. Returns zero vector if norm is ~0.
Vec3 normalise(const Vec3& vec) {
    float n = vec.norm();
    if (n > 0.0f) return vec / n;
    return vec;
}

// Normalize each row of a vector array in-place.
void normalise_inplace(std::vector<Vec3>& vecs) {
    for (auto& v : vecs) v = normalise(v);
}


// Generates n_pts uniformly distributed points on a sphere of radius r
// using the Fibonacci / golden-ratio spiral method.
std::vector<Vec3> generate_points_on_sphere(int n_pts = 100, float r = 1.0f) {
    const float goldenRatio = (1.0f + std::sqrt(5.0f)) / 2.0f;
    std::vector<Vec3> pts(n_pts);

    for (int i = 0; i < n_pts; ++i) {
        float idx   = static_cast<float>(i);
        float theta = 2.0f * M_PI * idx / goldenRatio;
        float phi   = std::acos(1.0f - 2.0f * idx / static_cast<float>(n_pts));
        pts[i] = {
            std::cos(theta) * std::sin(phi) * r,
            std::sin(theta) * std::sin(phi) * r,
            std::cos(phi) * r
        };
    }
    return pts;
}



// CGAL kernel and types
using K         = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3   = K::Point_3;
using Mesh      = CGAL::Surface_mesh<Point_3>;
using VDesc     = Mesh::Vertex_index;
using FDesc     = Mesh::Face_index;


// Helper: convert our Vec3 to a CGAL Point_3
Point_3 to_cgal(const Vec3& v) {
    return Point_3(v.x, v.y, v.z);
}

// Helper: convert a CGAL Point_3 back to Vec3
Vec3 from_cgal(const Point_3& p) {
    return Vec3(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z())
    );
}


ConvexHullResult compute_convex_hull(const std::vector<Vec3>& points) {
    // Convert input points to CGAL format
    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(points.size());
    for (const auto& p : points)
        cgal_pts.push_back(to_cgal(p));

    // Run CGAL's 3D convex hull — result is a Surface_mesh
    Mesh hull_mesh;
    CGAL::convex_hull_3(cgal_pts.begin(), cgal_pts.end(), hull_mesh);

    // --- Map CGAL vertex descriptors to contiguous integer indices ---
    // CGAL's Surface_mesh uses opaque vertex handles, not plain ints,
    // so we build an explicit handle→index map here.
    std::map<VDesc, int> vertex_index;
    int idx = 0;
    for (VDesc vd : hull_mesh.vertices())
        vertex_index[vd] = idx++;

    // Extract triangular faces
    // CGAL's convex hull always produces triangulated output, so each
    // face has exactly 3 half-edges / vertices.
    ConvexHullResult result;
    for (FDesc fd : hull_mesh.faces()) {
        Face tri;
        int k = 0;
        for (VDesc vd : CGAL::vertices_around_face(hull_mesh.halfedge(fd), hull_mesh)) {
            if (k >= 3) throw std::runtime_error(
                "compute_convex_hull: unexpected non-triangular face from CGAL.");
            tri[k++] = vertex_index.at(vd);
        }
        result.simplices.push_back(tri);
    }

    return result;
}


// Full generate_polyhedron using CGAL
DropletMesh generate_polyhedron(int n_vertices = 40, float radius = 1.0f) {
    // 1. Generate sphere-distributed seed points
    std::vector<Vec3> pts = generate_points_on_sphere(n_vertices, radius);

    // 2. Compute convex hull via CGAL
    //    Note: the hull may have fewer vertices than n_vertices if some
    //    input points are interior to the hull — matching Python behaviour.
    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(pts.size());
    for (const auto& p : pts)
        cgal_pts.push_back(to_cgal(p));

    Mesh hull_mesh;
    CGAL::convex_hull_3(cgal_pts.begin(), cgal_pts.end(), hull_mesh);

    // 3. Rebuild vertex array from hull mesh
    //    (hull vertices == pts in this case since all sphere points are
    //    on the convex hull, but we read back from the mesh to be safe)
    std::map<VDesc, int> vertex_index;
    std::vector<Vec3> hull_vertices;
    hull_vertices.reserve(hull_mesh.number_of_vertices());

    int i = 0;
    for (VDesc vd : hull_mesh.vertices()) {
        vertex_index[vd] = i++;
        hull_vertices.push_back(from_cgal(hull_mesh.point(vd)));
    }

    // 4. Extract faces and build neighbour lists simultaneously
    std::vector<Face>            faces;
    std::map<int, std::set<int>> nb_sets;

    for (int vi = 0; vi < static_cast<int>(hull_vertices.size()); ++vi)
        nb_sets[vi];  // ensure every vertex has an entry

    for (FDesc fd : hull_mesh.faces()) {
        Face tri;
        int k = 0;
        for (VDesc vd : CGAL::vertices_around_face(hull_mesh.halfedge(fd), hull_mesh))
            tri[k++] = vertex_index.at(vd);

        faces.push_back(tri);

        // Register edges in both directions
        for (int e = 0; e < 3; ++e) {
            int a = tri[e], b = tri[(e + 1) % 3];
            nb_sets[a].insert(b);
            nb_sets[b].insert(a);
        }
    }

    // 5. Convert neighbour sets to sorted vectors
    std::map<int, std::vector<int>> neighbours;
    for (auto& [id, s] : nb_sets)
        neighbours[id] = std::vector<int>(s.begin(), s.end());
        // std::set is already ordered, so the vectors are sorted — matching
        // Python's sorted(list(nbs)).

    return DropletMesh{hull_vertices, faces, neighbours};
}



// Nested map type mirroring Python's defaultdict(dict):
// weights[i][j] = w_ij
using WeightMap = std::unordered_map<int, std::unordered_map<int, float>>;


// --- Vec3 helpers used throughout this section ---

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}


// Accumulate w[i][j] += val, inserting 0.0 first if the key is absent.
// Mirrors Python's:  w[i].get(j, 0.0) + val
static inline void accum(WeightMap& w, int i, int j, float val) {
    auto& row = w[i];
    auto  it  = row.find(j);
    if (it == row.end()) row[j]  = val;
    else                 it->second += val;
}


// Computes cotangent weights w_ij to approximate the Laplace-Beltrami operator.
// Mirrors: compute_cotangent_weights(x, faces)
WeightMap compute_cotangent_weights(
    const std::vector<Vec3>& x,
    const std::vector<Face>& faces)
{
    WeightMap w;
    const float EPS = 1e-5f;

    for (const Face& f : faces) {
        const int   i0 = f[0], i1 = f[1], i2 = f[2];
        const Vec3& p0 = x[i0];
        const Vec3& p1 = x[i1];
        const Vec3& p2 = x[i2];

        // Edge vectors from each vertex
        Vec3 e0_1 = p1 - p0,  e0_2 = p2 - p0;   // edges at vertex 0
        Vec3 e1_2 = p2 - p1,  e1_0 = p0 - p1;   // edges at vertex 1
        Vec3 e2_0 = p0 - p2,  e2_1 = p1 - p2;   // edges at vertex 2

        // Cross products (their norms equal twice the sub-triangle area)
        Vec3 cr0 = cross(e0_1, e0_2);
        Vec3 cr1 = cross(e1_2, e1_0);
        Vec3 cr2 = cross(e2_0, e2_1);

        float n0 = cr0.norm();
        float n1 = cr1.norm();
        float n2 = cr2.norm();

        // Guard against degenerate (zero-area) faces — mirrors np.where(norm==0, 1e-5, norm)
        if (n0 == 0.0f) n0 = EPS;
        if (n1 == 0.0f) n1 = EPS;
        if (n2 == 0.0f) n2 = EPS;

        // cot(angle) = dot(edges) / |cross(edges)|
        float cot0 = dot(e0_1, e0_2) / n0;
        float cot1 = dot(e1_2, e1_0) / n1;
        float cot2 = dot(e2_0, e2_1) / n2;

        // Assign weights; clamp negatives to 0 (matches Python's max(0.0, ...))
        // w0 is the weight on edge (i0,i1), opposite to vertex i2 → uses cot2
        float w0 = std::max(0.0f, cot2 * 0.5f);
        float w1 = std::max(0.0f, cot0 * 0.5f);
        float w2 = std::max(0.0f, cot1 * 0.5f);

        accum(w, i0, i1, w0);  accum(w, i1, i0, w0);
        accum(w, i1, i2, w1);  accum(w, i2, i1, w1);
        accum(w, i2, i0, w2);  accum(w, i0, i2, w2);
    }

    return w;
}


// Result type for compute_vertex_normals_and_volume
struct NormalsAndVolume {
    std::vector<Vec3> normals;   // unit outward normal per vertex
    float             volume;    // absolute enclosed volume
};

// Calculates per-vertex normals and the exact enclosed volume of the mesh.
// Mirrors: compute_vertex_normals_and_volume(x, faces)
NormalsAndVolume compute_vertex_normals_and_volume(
    const std::vector<Vec3>& x,
    const std::vector<Face>& faces)
{
    const int N = static_cast<int>(x.size());
    std::vector<Vec3> n(N, {0.0f, 0.0f, 0.0f});
    float V = 0.0f;

    // Centre of mass — mirrors np.mean(x, axis=0)
    Vec3 com = {0.0f, 0.0f, 0.0f};
    for (const auto& v : x) { com.x += v.x; com.y += v.y; com.z += v.z; }
    com = com / static_cast<float>(N);

    for (const Face& f : faces) {
        int i0 = f[0], i1 = f[1], i2 = f[2];
        Vec3 p0 = x[i0], p1 = x[i1], p2 = x[i2];

        Vec3 n_face = cross(p1 - p0, p2 - p0);

        // Flip face if it points inward — mirrors: if dot(n_face, x0-com) < 0
        if (dot(n_face, p0 - com) < 0.0f) {
            n_face = n_face * -1.0f;
            std::swap(p1, p2);          // swap so the volume term stays consistent
        }

        n[i0].x += n_face.x;  n[i0].y += n_face.y;  n[i0].z += n_face.z;
        n[i1].x += n_face.x;  n[i1].y += n_face.y;  n[i1].z += n_face.z;
        n[i2].x += n_face.x;  n[i2].y += n_face.y;  n[i2].z += n_face.z;

        // Divergence theorem: V += dot(p0-com, cross(p1-com, p2-com)) / 6
        V += dot(p0 - com, cross(p1 - com, p2 - com)) / 6.0f;
    }

    // Normalise accumulated vertex normals — mirrors n / where(norms==0, 1.0, norms)
    for (auto& ni : n) {
        float nm = ni.norm();
        if (nm > 0.0f) { ni.x /= nm; ni.y /= nm; ni.z /= nm; }
        // if nm == 0, leave as zero vector (matches Python's division-by-1 fallback)
    }

    return {n, std::abs(V)};
}


// Computes positional (Δx) and velocity (Δv) Laplacians for all vertices.
// Mirrors: compute_laplacians(x, v, neighbours, w)
struct Laplacians {
    std::vector<Vec3> delta_x;
    std::vector<Vec3> delta_v;
};

Laplacians compute_laplacians(
    const std::vector<Vec3>&             x,
    const std::vector<Vec3>&             v,
    const std::map<int, std::vector<int>>& neighbours,
    const WeightMap&                     w)
{
    const int N = static_cast<int>(x.size());
    std::vector<Vec3> dx(N, {0,0,0});
    std::vector<Vec3> dv(N, {0,0,0});

    for (int i = 0; i < N; ++i) {
        float sum_w = 0.0f;

        for (int j : neighbours.at(i)) {
            // Mirrors:  w[i].get(j, 1.0) — fall back to 1.0 if weight absent
            float w_ij = 1.0f;
            auto row = w.find(i);
            if (row != w.end()) {
                auto col = row->second.find(j);
                if (col != row->second.end()) w_ij = col->second;
            }

            Vec3 diff_x = x[j] - x[i];
            Vec3 diff_v = v[j] - v[i];

            dx[i].x += w_ij * diff_x.x;
            dx[i].y += w_ij * diff_x.y;
            dx[i].z += w_ij * diff_x.z;

            dv[i].x += w_ij * diff_v.x;
            dv[i].y += w_ij * diff_v.y;
            dv[i].z += w_ij * diff_v.z;

            sum_w += w_ij;
        }

        if (sum_w > 0.0f) {
            dx[i] = dx[i] / sum_w;
            dv[i] = dv[i] / sum_w;
        }
    }

    return {dx, dv};
}



// ─────────────────────────────────────────────────────────────────────────────
// apply_surface_interactions
// Handles Z=0 plane collisions, boundary constraints, and sliding friction.
// ─────────────────────────────────────────────────────────────────────────────

struct SurfaceResult {
    std::vector<Vec3> x_new;
    std::vector<Vec3> v_new;
};

SurfaceResult apply_surface_interactions(
    const std::vector<Vec3>& x,
    const std::vector<Vec3>& v,
    float dt,
    float friction_coeff,
    float adhesion_dist)
{
    std::vector<Vec3> x_new = x;   // copy — mirrors x.copy()
    std::vector<Vec3> v_new = v;

    for (int i = 0; i < static_cast<int>(x_new.size()); ++i) {
        if (x_new[i].z <= adhesion_dist) {

            // Hard floor constraint
            if (x_new[i].z < 0.0f) {
                x_new[i].z = 0.0f;
                if (v_new[i].z < 0.0f)
                    v_new[i].z *= -0.2f;   // partial restitution
            }

            // Coulomb-style sliding friction on the horizontal (XY) component
            float vx = v_new[i].x, vy = v_new[i].y;
            float speed = std::sqrt(vx * vx + vy * vy);   // np.linalg.norm(v_horiz)

            if (speed > 0.0f) {
                float drop = friction_coeff * dt;
                if (speed < drop) {
                    v_new[i].x = 0.0f;
                    v_new[i].y = 0.0f;
                } else {
                    float scale = (speed - drop) / speed;
                    v_new[i].x *= scale;
                    v_new[i].y *= scale;
                }
            }

            // Additional viscous surface slip drag
            float slip_scale = 1.0f - 0.05f * dt;
            v_new[i].x *= slip_scale;
            v_new[i].y *= slip_scale;
        }
    }

    return {x_new, v_new};
}


// ─────────────────────────────────────────────────────────────────────────────
// boundary_force
// Computes per-vertex contact-angle hysteresis boundary force.
// ─────────────────────────────────────────────────────────────────────────────

// Mirrors the default ground_normal = [0, 0, 1]
static const Vec3 DEFAULT_GROUND_NORMAL = {0.0f, 0.0f, 1.0f};

// contact_mask[i] == true  →  vertex i is in contact with the surface
std::vector<Vec3> boundary_force(
    float                     alpha,
    const std::vector<Vec3>&  n_l,
    const std::vector<bool>&  contact_mask,
    float                     receding_angle,
    float                     advancing_angle,
    const Vec3&               ground_normal = DEFAULT_GROUND_NORMAL)
{
    if (receding_angle > advancing_angle)
        throw std::invalid_argument(
            "boundary_force: receding_angle must be <= advancing_angle");

    const int N = static_cast<int>(n_l.size());
    std::vector<Vec3> f_boundary(N, {0.0f, 0.0f, 0.0f});

    // Early exit if no contact vertices — mirrors np.count_nonzero(contact_mask)==0
    bool any_contact = false;
    for (bool c : contact_mask) if (c) { any_contact = true; break; }
    if (!any_contact) return f_boundary;

    const Vec3 n_i = normalise(ground_normal);

    for (int i = 0; i < N; ++i) {
        if (!contact_mask[i]) continue;

        const Vec3& n_li = n_l[i];

        // Angle between vertex normal and ground normal
        float dot_val = std::clamp(dot(n_li, n_i), -1.0f, 1.0f);
        float angle   = std::acos(dot_val);

        // Tangential (projected) component of n_li perpendicular to ground_normal
        // n_p = n_li - dot_val * n_i
        Vec3 n_p = {
            n_li.x - dot_val * n_i.x,
            n_li.y - dot_val * n_i.y,
            n_li.z - dot_val * n_i.z
        };
        Vec3 n_p_dir = normalise(n_p);

        // Skip zero-length tangential component — mirrors: if norm(n_p_dir)==0: continue
        if (n_p_dir.norm() < 1e-8f) continue;

        // Contact-angle hysteresis:
        //   inside the hysteresis band  → no force
        //   below receding angle        → pull inward  (angle - receding_angle < 0)
        //   above advancing angle       → push outward (angle - advancing_angle > 0)
        if (angle > receding_angle && angle < advancing_angle) continue;

        float delta = (angle <= receding_angle)
                    ? (angle - receding_angle)    // negative  → inward pull
                    : (angle - advancing_angle);  // positive  → outward push

        f_boundary[i] = {
            alpha * delta * n_p_dir.x,
            alpha * delta * n_p_dir.y,
            alpha * delta * n_p_dir.z
        };
    }

    return f_boundary;
}


// ─────────────────────────────────────────────────────────────────────────────
// compute_total_acceleration
// Computes total acceleration and velocity Laplacian for a given state.
// ─────────────────────────────────────────────────────────────────────────────

struct AccelerationResult {
    std::vector<Vec3> a_total;
    std::vector<Vec3> delta_v;
};

// Sanitise a Vec3: replace NaN / ±Inf with zero — mirrors np.nan_to_num(..., nan=0, posinf=0, neginf=0)
static inline Vec3 sanitise(const Vec3& v) {
    auto clean = [](float f) -> float {
        return (std::isfinite(f)) ? f : 0.0f;
    };
    return {clean(v.x), clean(v.y), clean(v.z)};
}

AccelerationResult compute_total_acceleration(
    const std::vector<Vec3>&              x,
    const std::vector<Vec3>&              v,
    const std::vector<Face>&              faces,
    const std::map<int, std::vector<int>>& neighbours,
    float                                 V_0,
    const Vec3&                           g,
    float                                 gamma,
    float                                 k_v,
    float                                 boundary_alpha,
    float                                 receding_angle,
    float                                 advancing_angle,
    float                                 adhesion_dist,
    float                                 max_internal_accel,
    float                                 density,
    float                                 damping_gain = 1.0f)
{
    const int N = static_cast<int>(x.size());

    // Geometry quantities
    WeightMap        w        = compute_cotangent_weights(x, faces);
    auto [n, V]               = compute_vertex_normals_and_volume(x, faces);
    auto [delta_x, delta_v]   = compute_laplacians(x, v, neighbours, w);

    // Contact mask: vertex is "touching" the surface if z <= adhesion_dist
    std::vector<bool> contact_mask(N);
    for (int i = 0; i < N; ++i)
        contact_mask[i] = (x[i].z <= adhesion_dist);

    // Internal force contributions
    //   f_st  : surface tension  = gamma * Δx   (mean-curvature flow)
    //   f_vol : volume pressure  = k_v*(V0-V)*n  (volume preservation)
    //   f_b   : boundary force   (contact-angle hysteresis)
    std::vector<Vec3> f_b = boundary_force(
        boundary_alpha, n, contact_mask, receding_angle, advancing_angle);

    std::vector<Vec3> a_total(N);

    for (int i = 0; i < N; ++i) {
        // f_st = gamma * delta_x[i]
        Vec3 f_st = {
            gamma * delta_x[i].x,
            gamma * delta_x[i].y,
            gamma * delta_x[i].z
        };

        // f_vol = k_v * (V_0 - V) * n[i]
        float vp = k_v * (V_0 - V);
        Vec3 f_vol = {vp * n[i].x, vp * n[i].y, vp * n[i].z};

        // a_internal = (f_st + f_vol + f_boundary) / density
        Vec3 a_int = {
            (f_st.x + f_vol.x + f_b[i].x) / density,
            (f_st.y + f_vol.y + f_b[i].y) / density,
            (f_st.z + f_vol.z + f_b[i].z) / density
        };

        // Clamp internal acceleration magnitude to max_internal_accel
        // scale = min(1, max_internal_accel / max(|a_int|, 1e-8))
        float a_int_norm = a_int.norm();
        float scale = std::min(1.0f, max_internal_accel
                                     / std::max(a_int_norm, 1e-8f));
        a_int = a_int * scale;

        // a_total = g + a_internal   (g broadcast to every vertex)
        a_total[i] = sanitise({
            g.x + a_int.x,
            g.y + a_int.y,
            g.z + a_int.z
        });

        delta_v[i] = sanitise(delta_v[i]);
    }

    return {a_total, delta_v};
}



// ─────────────────────────────────────────────────────────────────────────────
// Parameter structs
// Grouping defaults mirrors Python's keyword arguments with default values.
// ─────────────────────────────────────────────────────────────────────────────

struct EulerParams {
    float boundary_alpha    = 0.5f;
    float receding_angle    = 70.0f  * (M_PI / 180.0f);   // np.deg2rad(70)
    float advancing_angle   = 95.0f  * (M_PI / 180.0f);   // np.deg2rad(95)
    float adhesion_dist     = 0.05f;
    float max_internal_accel = 200.0f;
    float density           = 1.0f;
    float damping_gain      = 1.0f;
};

struct SimParams {
    Vec3  g                 = {0.0f, 0.0f, -9.81f};
    float gamma             = 0.5f;
    float mu                = 0.3f;
    float eta               = 0.05f;
    float k_v               = 2000.0f;
    float density           = 1.0f;
    float rho               = 1.0f;
    float boundary_alpha    = 0.5f;
    float alpha             = 0.5f;
    float friction_coeff    = 0.5f;
    float receding_angle    = 70.0f  * (M_PI / 180.0f);
    float advancing_angle   = 95.0f  * (M_PI / 180.0f);
    float adhesion_dist     = 0.05f;
    float max_internal_accel = 200.0f;
    float damping_gain      = 1.0f;

    // Mirrors Python's optional contact_angles tuple: (receding, advancing).
    // Leave as std::nullopt to use receding_angle / advancing_angle above.
    std::optional<std::pair<float,float>> contact_angles = std::nullopt;
};


// ─────────────────────────────────────────────────────────────────────────────
// Step result
// ─────────────────────────────────────────────────────────────────────────────

struct StepResult {
    std::vector<Vec3> v_new;
    std::vector<Vec3> x_new;
};


// ─────────────────────────────────────────────────────────────────────────────
// step_forward_euler
// Advances the simulation by one explicit Euler time step.
// ─────────────────────────────────────────────────────────────────────────────

StepResult step_forward_euler(
    const std::vector<Vec3>&               v,
    const std::vector<Vec3>&               x,
    const std::vector<Face>&               faces,
    const std::map<int, std::vector<int>>& neighbours,
    float                                  V_0,
    float                                  dt,
    const Vec3&                            g,
    float                                  gamma,
    float                                  mu,
    float                                  eta,
    float                                  k_v,
    float                                  friction_coeff,
    const EulerParams&                     ep = EulerParams{})
{
    const int N = static_cast<int>(x.size());

    // ── 1. Accelerations ────────────────────────────────────────────────────
    auto [a_total, delta_v] = compute_total_acceleration(
        x, v, faces, neighbours,
        V_0, g, gamma, k_v,
        ep.boundary_alpha, ep.receding_angle, ep.advancing_angle,
        ep.adhesion_dist, ep.max_internal_accel, ep.density, ep.damping_gain);

    // ── 2. Base viscosity + Laplacian smoothing ──────────────────────────────
    // v_damped = (1 - mu*dt)*v + (eta*dt)*delta_v
    std::vector<Vec3> v_damped(N);
    float base_scale  = 1.0f - mu * dt;
    float delta_scale = eta * dt;

    for (int i = 0; i < N; ++i) {
        v_damped[i] = {
            base_scale * v[i].x + delta_scale * delta_v[i].x,
            base_scale * v[i].y + delta_scale * delta_v[i].y,
            base_scale * v[i].z + delta_scale * delta_v[i].z
        };
    }

    // ── 3. Energy-aware damping ──────────────────────────────────────────────
    // lin  = max(0, mu)  * damping_gain
    // quad = (0.2 + 0.3*max(0, eta)) * damping_gain
    // damping[i] = 1 / (1 + lin*dt + quad*speed[i]*dt)
    float lin  = std::max(0.0f, mu)  * ep.damping_gain;
    float quad = (0.2f + 0.3f * std::max(0.0f, eta)) * ep.damping_gain;

    float speed_sum = 0.0f;

    for (int i = 0; i < N; ++i) {
        float speed   = std::sqrt(v[i].x*v[i].x + v[i].y*v[i].y + v[i].z*v[i].z);
        float damping = 1.0f / (1.0f + lin * dt + quad * speed * dt);
        v_damped[i]   = v_damped[i] * damping;
        speed_sum    += speed;
    }

    // ── 4. Residual global damping in low-speed regime ───────────────────────
    // Ensures eventual settling when mean speed < 0.25.
    float mean_speed = speed_sum / static_cast<float>(N);
    if (mean_speed < 0.25f) {
        float settle_scale = std::max(0.0f, 1.0f - 0.3f * dt * ep.damping_gain);
        for (auto& vd : v_damped) vd = vd * settle_scale;
    }

    // ── 5. Euler integration + velocity clamp ────────────────────────────────
    // v_new = clip(v_damped + a_total * dt, -15, 15)
    // x_new = x + v_new * dt
    std::vector<Vec3> v_new(N), x_new(N);

    auto vclamp = [](float f) { return std::clamp(f, -15.0f, 15.0f); };

    for (int i = 0; i < N; ++i) {
        v_new[i] = {
            vclamp(v_damped[i].x + a_total[i].x * dt),
            vclamp(v_damped[i].y + a_total[i].y * dt),
            vclamp(v_damped[i].z + a_total[i].z * dt)
        };
        x_new[i] = {
            x[i].x + v_new[i].x * dt,
            x[i].y + v_new[i].y * dt,
            x[i].z + v_new[i].z * dt
        };
    }

    // ── 6. Surface collision and friction ────────────────────────────────────
    auto [x_final, v_final] = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, ep.adhesion_dist);

    return {v_final, x_final};
}


// ─────────────────────────────────────────────────────────────────────────────
// step_simulation
// Unified simulation step wrapper — resolves all optional / override params
// then delegates to step_forward_euler.
// ─────────────────────────────────────────────────────────────────────────────

StepResult step_simulation(
    const std::vector<Vec3>&               v,
    const std::vector<Vec3>&               x,
    const std::vector<Face>&               faces,
    const std::map<int, std::vector<int>>& neighbours,
    float                                  V_0,
    float                                  dt,
    const SimParams&                       sp = SimParams{})
{
    // alpha_eff: alpha takes precedence over boundary_alpha (mirrors Python's
    // "alpha if alpha is not None else boundary_alpha" — here both always have
    // values, so we just use alpha as the primary parameter, matching that intent).
    float alpha_eff   = sp.alpha;

    // density_eff: rho takes precedence over density
    float density_eff = sp.rho;

    // contact_angles override individual receding/advancing angles when set
    float receding_eff  = sp.receding_angle;
    float advancing_eff = sp.advancing_angle;
    if (sp.contact_angles.has_value()) {
        receding_eff  = sp.contact_angles->first;
        advancing_eff = sp.contact_angles->second;
    }

    EulerParams ep;
    ep.boundary_alpha     = alpha_eff;
    ep.receding_angle     = receding_eff;
    ep.advancing_angle    = advancing_eff;
    ep.adhesion_dist      = sp.adhesion_dist;
    ep.max_internal_accel = sp.max_internal_accel;
    ep.density            = density_eff;
    ep.damping_gain       = sp.damping_gain;

    return step_forward_euler(
        v, x, faces, neighbours,
        V_0, dt,
        sp.g, sp.gamma, sp.mu, sp.eta, sp.k_v,
        sp.friction_coeff,
        ep);
}