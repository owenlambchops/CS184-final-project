#include "wd/sim/droplet_template.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
#include <numbers>

namespace wd {

namespace {
    // Helper to generate a unique key for an edge regardless of vertex order
    int64_t getEdgeKey(int i1, int i2) {
        int64_t min_i = std::min(i1, i2);
        int64_t max_i = std::max(i1, i2);
        return (min_i << 32) | max_i;
    }

    // Stores adjacency data needed for the Loop Subdivision stencil
    struct Edge {
        int v1 = -1, v2 = -1;
        int opposite1 = -1, opposite2 = -1;
        int new_vid = -1; // The index of the newly inserted vertex
    };

    // Safely tracks the two opposite vertices for the two triangles sharing an edge
    void addEdgeOpposite(std::map<int64_t, Edge>& edges, int v1, int v2, int opposite) {
        int64_t key = getEdgeKey(v1, v2);
        if (edges.find(key) == edges.end()) {
            edges[key].v1 = v1;
            edges[key].v2 = v2;
            edges[key].opposite1 = opposite;
        } else {
            edges[key].opposite2 = opposite; // Fill the second triangle's opposite vertex
        }
    }
}

/**
 * @brief Generates a highly uniform spherical physics mesh using Loop Subdivision.
 * * Charles Loop's subdivision algorithm iteratively splits triangles while applying 
 * a specific relaxation stencil to both old and new vertices. This produces a 
 * beautifully uniform limit surface. We run this algorithm on a base triangulated 
 * cube (forming a quad-sphere topology) and perfectly project it onto a sphere.
 * * @param subdivisions Number of times to subdivide.
 * @param radius Radius of the generated sphere.
 */
std::shared_ptr<DropletTemplate> DropletTemplate::CreateSphericalMesh(
    int subdivisions, double radius) {
    
    auto tpl = std::make_shared<DropletTemplate>();
    
    // Clamp subdivisions to prevent accidental memory exhaustion
    subdivisions = std::clamp(subdivisions, 0, 6);
    
    Eigen::MatrixXd V(8, 3);
    auto addBaseVertex = [&](int idx, double x, double y, double z) {
        Vec3 v(x, y, z);
        V.row(idx) = v.normalized().transpose();
    };

    // 8 Base Vertices (Cube)
    addBaseVertex(0, -1, -1, -1);
    addBaseVertex(1,  1, -1, -1);
    addBaseVertex(2,  1,  1, -1);
    addBaseVertex(3, -1,  1, -1);
    addBaseVertex(4, -1, -1,  1);
    addBaseVertex(5,  1, -1,  1);
    addBaseVertex(6,  1,  1,  1);
    addBaseVertex(7, -1,  1,  1);

    // 12 Base Faces (Triangulated Cube)
    std::vector<Eigen::RowVector3i> F;
    F.reserve(12 * std::pow(4, subdivisions));
    
    F.push_back({4, 5, 6}); F.push_back({4, 6, 7}); // Front
    F.push_back({1, 0, 3}); F.push_back({1, 3, 2}); // Back
    F.push_back({0, 4, 7}); F.push_back({0, 7, 3}); // Left
    F.push_back({5, 1, 2}); F.push_back({5, 2, 6}); // Right
    F.push_back({7, 6, 2}); F.push_back({7, 2, 3}); // Top
    F.push_back({0, 1, 5}); F.push_back({0, 5, 4}); // Bottom

    // Loop Subdivision iterations
    for (int sub = 0; sub < subdivisions; ++sub) {
        std::map<int64_t, Edge> edges;
        std::vector<std::vector<int>> adj(V.rows());

        // 1. Build Adjacency and Edge/Face mappings
        for (const auto& tri : F) {
            int a = tri(0), b = tri(1), c = tri(2);
            
            addEdgeOpposite(edges, a, b, c);
            addEdgeOpposite(edges, b, c, a);
            addEdgeOpposite(edges, c, a, b);

            auto addUnique = [](std::vector<int>& list, int val) {
                if (std::find(list.begin(), list.end(), val) == list.end()) list.push_back(val);
            };
            addUnique(adj[a], b); addUnique(adj[a], c);
            addUnique(adj[b], a); addUnique(adj[b], c);
            addUnique(adj[c], a); addUnique(adj[c], b);
        }

        Eigen::MatrixXd next_V(V.rows() + edges.size(), 3);

        // 2. Loop Rule for EXISTING (Even) Vertices
        for (int i = 0; i < V.rows(); ++i) {
            int n = adj[i].size();
            double beta;
            
            // Charles Loop's original weighting scheme
            if (n > 3) {
                beta = (1.0 / n) * (5.0 / 8.0 - std::pow(3.0 / 8.0 + 0.25 * std::cos(2.0 * std::numbers::pi / n), 2));
            } else {
                beta = 3.0 / 16.0;
            }

            Vec3 sum_adj = Vec3::Zero();
            for (int nb : adj[i]) {
                sum_adj += V.row(nb).transpose();
            }

            Vec3 vi = V.row(i).transpose();
            Vec3 new_vi = (1.0 - n * beta) * vi + beta * sum_adj;
            next_V.row(i) = new_vi.transpose();
        }

        // 3. Loop Rule for NEW (Odd) Vertices created on edges
        int new_vIdx = V.rows();
        for (auto& pair : edges) {
            Edge& e = pair.second;
            Vec3 v1 = V.row(e.v1).transpose();
            Vec3 v2 = V.row(e.v2).transpose();

            if (e.opposite2 != -1) {
                // Interior edge: weighted by endpoints and opposite vertices
                Vec3 v3 = V.row(e.opposite1).transpose();
                Vec3 v4 = V.row(e.opposite2).transpose();
                Vec3 new_v = (3.0 / 8.0) * (v1 + v2) + (1.0 / 8.0) * (v3 + v4);
                next_V.row(new_vIdx) = new_v.transpose();
            } else {
                // Fallback for boundaries (Icosahedrons have none, but here for safety)
                next_V.row(new_vIdx) = (0.5 * (v1 + v2)).transpose();
            }

            e.new_vid = new_vIdx++;
        }

        // 4. Stitch together the new topology (1 triangle becomes 4)
        std::vector<Eigen::RowVector3i> next_F;
        next_F.reserve(F.size() * 4);
        
        for (const auto& tri : F) {
            int a = tri(0), b = tri(1), c = tri(2);
            int e1 = edges[getEdgeKey(a, b)].new_vid;
            int e2 = edges[getEdgeKey(b, c)].new_vid;
            int e3 = edges[getEdgeKey(c, a)].new_vid;

            next_F.push_back({a, e1, e3});
            next_F.push_back({b, e2, e1});
            next_F.push_back({c, e3, e2});
            next_F.push_back({e1, e2, e3});
        }

        V = next_V;
        F = next_F;
    }

    // Finalization: Project relaxed mesh perfectly onto the mathematical sphere
    tpl->restV_.resize(V.rows(), 3);
    for (int i = 0; i < V.rows(); ++i) {
        tpl->restV_.row(i) = (V.row(i).normalized() * radius).transpose();
    }

    tpl->F_.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        tpl->F_.row(i) = F[i];
    }

    // A fully closed sphere has no boundary loop
    tpl->boundaryLoop_.clear();
    
    return tpl;
}

} // namespace wd