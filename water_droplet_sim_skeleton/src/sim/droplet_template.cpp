#include "wd/sim/droplet_template.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <tuple>
#include <vector>

namespace wd {
namespace {

int64_t getEdgeKey(int i1, int i2) {
    const int64_t lo = std::min(i1, i2);
    const int64_t hi = std::max(i1, i2);
    return (lo << 32) | hi;
}

std::vector<Eigen::Vector2i> buildUniqueEdges(const MatX3i& faces) {
    std::set<std::pair<int, int>> edgeSet;
    auto add = [&](int a, int b) {
        if (a > b) std::swap(a, b);
        edgeSet.emplace(a, b);
    };

    for (int i = 0; i < faces.rows(); ++i) {
        const int a = faces(i, 0);
        const int b = faces(i, 1);
        const int c = faces(i, 2);
        add(a, b);
        add(b, c);
        add(c, a);
    }

    std::vector<Eigen::Vector2i> edges;
    edges.reserve(edgeSet.size());
    for (const auto& e : edgeSet) edges.emplace_back(e.first, e.second);
    return edges;
}

void addEdgeOpposite(std::map<int64_t, Edge>& edges, int v1, int v2, int opposite) {
    const int64_t key = getEdgeKey(v1, v2);
    auto it = edges.find(key);
    if (it == edges.end()) {
        edges[key].v1 = v1;
        edges[key].v2 = v2;
        edges[key].opposite1 = opposite;
    } else {
        it->second.opposite2 = opposite;
    }
}

} // namespace

std::shared_ptr<DropletTemplate> DropletTemplate::CreateSphericalMesh(int subdivisions, double radius) {
    subdivisions = std::clamp(subdivisions, 0, 4);

    // Fibonacci point count growth (kept moderate because hull is O(n^4) brute-force here).
    const int nPts = 24 * (1 << subdivisions);
    MatX3d V(nPts, 3);
    const double golden = (1.0 + std::sqrt(5.0)) * 0.5;
    for (int i = 0; i < nPts; ++i) {
        const double t = static_cast<double>(i) + 0.5;
        const double z = 1.0 - 2.0 * t / static_cast<double>(nPts);
        const double rxy = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double theta = 2.0 * std::numbers::pi * t / golden;
        V(i, 0) = rxy * std::cos(theta) * radius;
        V(i, 1) = rxy * std::sin(theta) * radius;
        V(i, 2) = z * radius;
    }

    // Brute-force convex hull triangle detection.
    // Face (i,j,k) is on hull if all other points lie on one side of its plane.
    constexpr double eps = 1e-9;
    std::vector<Eigen::Vector3i> faces;
    std::set<std::tuple<int, int, int>> uniqueFaces;
    faces.reserve(2 * nPts);

    for (int i = 0; i < nPts; ++i) {
        const Vec3 pi = V.row(i).transpose();
        for (int j = i + 1; j < nPts; ++j) {
            const Vec3 pj = V.row(j).transpose();
            for (int k = j + 1; k < nPts; ++k) {
                const Vec3 pk = V.row(k).transpose();
                Vec3 nrm = (pj - pi).cross(pk - pi);
                const double nn = nrm.norm();
                if (nn < eps) continue;

                bool hasPos = false;
                bool hasNeg = false;
                for (int m = 0; m < nPts; ++m) {
                    if (m == i || m == j || m == k) continue;
                    const Vec3 pm = V.row(m).transpose();
                    const double s = nrm.dot(pm - pi);
                    if (s > eps) hasPos = true;
                    else if (s < -eps) hasNeg = true;
                    if (hasPos && hasNeg) break;
                }
                if (hasPos && hasNeg) continue;

                // Orient outward.
                int a = i, b = j, c = k;
                if (nrm.dot((pi + pj + pk) / 3.0) < 0.0) std::swap(b, c);
                const auto key = std::make_tuple(
                    std::min({a, b, c}),
                    a + b + c - std::min({a, b, c}) - std::max({a, b, c}),
                    std::max({a, b, c}));
                if (uniqueFaces.insert(key).second) faces.emplace_back(a, b, c);
            }
        }
    }

    MatX3i F(static_cast<int>(faces.size()), 3);
    for (int f = 0; f < static_cast<int>(faces.size()); ++f) {
        F.row(f) = faces[f];
    }

    auto tpl = std::shared_ptr<DropletTemplate>(new DropletTemplate());
    tpl->restV_ = V;
    tpl->F_ = F;
    tpl->E_ = buildUniqueEdges(tpl->F_);
    tpl->boundaryLoop_.clear();
    return tpl;
}

} // namespace wd
