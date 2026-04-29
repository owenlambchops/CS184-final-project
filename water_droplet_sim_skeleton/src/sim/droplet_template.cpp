#include "wd/sim/droplet_template.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <igl/convex_hull.h>

namespace wd {

namespace {
// Generates uniformly distributed points on sphere using Fibonacci sphere algorithm
MatX3d generatePointsOnSphere(int nPoints, double radius) {
    MatX3d points(nPoints, 3);
    double goldenRatio = (1.0 + std::sqrt(5.0)) / 2.0;
    for (int i = 0; i < nPoints; ++i) {
        double theta = 2.0 * std::numbers::pi * i / goldenRatio;
        double phi = std::acos(1.0 - 2.0 * i / static_cast<double>(nPoints));
        double x = std::cos(theta) * std::sin(phi);
        double y = std::sin(theta) * std::sin(phi);
        double z = std::cos(phi);
        points.row(i) = Vec3(x, y, z) * radius;
    }
    return points;
}
} // namespace


/**
 * Safely generates a libMesh sphere and extracts it into contiguous arrays
 * suited for real-time physics iteration.
 */
SimState generate_sphere_mesh(libMesh::Mesh& mesh, double radius, int segments) {
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
        state.x.push_back(Vec3((*node)(0), (*node)(1), (*node)(2)));
        state.v.push_back(Vec3(0, 0, 0));
        state.a.push_back(Vec3(0, 0, 0));
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

} // namespace wd
