#include "wd/sim/droplet_template.h"
#include "wd/sim/cgl_math.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <numbers>
#include <set>
#include <vector>

namespace wd {
namespace {

std::vector<Eigen::Vector2i> buildUniqueEdges(const MatX3i& faces) {
    std::set<std::pair<int, int>> edgeSet;
    auto add = [&](int a, int b) {
        if (a > b) std::swap(a, b);
        edgeSet.emplace(a, b);
    };
    for (int i = 0; i < faces.rows(); ++i) {
        add(faces(i, 0), faces(i, 1));
        add(faces(i, 1), faces(i, 2));
        add(faces(i, 2), faces(i, 0));
    }

    std::vector<Eigen::Vector2i> edges;
    edges.reserve(edgeSet.size());
    for (const auto& e : edgeSet) edges.emplace_back(e.first, e.second);
    return edges;
}

int intPow4(int e) {
    int out = 1;
    for (int i = 0; i < e; ++i) out *= 4;
    return out;
}

int64_t getEdgeKey(int i1, int i2) {
    const int64_t lo = std::min(i1, i2);
    const int64_t hi = std::max(i1, i2);
    return (lo << 32) | hi;
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
    subdivisions = std::clamp(subdivisions, 0, 6);

    std::vector<Vec3> vertices;
    vertices.reserve(8);
    auto addBaseVertex = [&](double x, double y, double z) {
        const Vec3 v(x, y, z);
        vertices.push_back(v.normalized());
    };

    addBaseVertex(-1, -1, -1);
    addBaseVertex(1, -1, -1);
    addBaseVertex(1, 1, -1);
    addBaseVertex(-1, 1, -1);
    addBaseVertex(-1, -1, 1);
    addBaseVertex(1, -1, 1);
    addBaseVertex(1, 1, 1);
    addBaseVertex(-1, 1, 1);

    std::vector<Face> faces;
    faces.reserve(static_cast<size_t>(12 * intPow4(subdivisions)));

    auto addFace = [&](int a, int b, int c) {
        faces.emplace_back(a, b, c);
    };

    addFace(4, 5, 6); addFace(4, 6, 7);
    addFace(1, 0, 3); addFace(1, 3, 2);
    addFace(0, 4, 7); addFace(0, 7, 3);
    addFace(5, 1, 2); addFace(5, 2, 6);
    addFace(7, 6, 2); addFace(7, 2, 3);
    addFace(0, 1, 5); addFace(0, 5, 4);

    for (int sub = 0; sub < subdivisions; ++sub) {
        std::map<int64_t, Edge> edges;
        for (const Face& f : faces) {
            const int a = f[0];
            const int b = f[1];
            const int c = f[2];
            addEdgeOpposite(edges, a, b, c);
            addEdgeOpposite(edges, b, c, a);
            addEdgeOpposite(edges, c, a, b);
        }

        std::vector<Vec3> nextV = vertices;
        nextV.reserve(vertices.size() + edges.size());

        for (auto& kv : edges) {
            Edge& e = kv.second;
            const Vec3 v1 = vertices[static_cast<size_t>(e.v1)];
            const Vec3 v2 = vertices[static_cast<size_t>(e.v2)];
            const Vec3 newV = 0.5 * (v1 + v2);
            nextV.push_back(newV.normalized());
            e.new_vid = static_cast<int>(nextV.size() - 1);
        }

        std::vector<Face> nextF;
        nextF.reserve(faces.size() * 4);
        for (const Face& f : faces) {
            const int a = f[0];
            const int b = f[1];
            const int c = f[2];
            const int e1 = edges[getEdgeKey(a, b)].new_vid;
            const int e2 = edges[getEdgeKey(b, c)].new_vid;
            const int e3 = edges[getEdgeKey(c, a)].new_vid;

            nextF.emplace_back(a, e1, e3);
            nextF.emplace_back(b, e2, e1);
            nextF.emplace_back(c, e3, e2);
            nextF.emplace_back(e1, e2, e3);
        }

        vertices = std::move(nextV);
        faces = std::move(nextF);
    }

    MatX3d V(static_cast<int>(vertices.size()), 3);
    for (int i = 0; i < static_cast<int>(vertices.size()); ++i) {
        V.row(i) = (vertices[static_cast<size_t>(i)].normalized() * radius).transpose();
    }
    MatX3i F(static_cast<int>(faces.size()), 3);
    for (int i = 0; i < static_cast<int>(faces.size()); ++i) F.row(i) = faces[static_cast<size_t>(i)];

    auto tpl = std::shared_ptr<DropletTemplate>(new DropletTemplate());
    tpl->restV_ = V;
    tpl->F_ = F;
    tpl->E_ = buildUniqueEdges(tpl->F_);
    tpl->boundaryLoop_.clear();
    return tpl;
}

} // namespace wd
