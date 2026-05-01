#include "wd/sim/droplet_template.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <numbers>
#include <vector>

namespace wd {
namespace {

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

    MatX3d V(8, 3);
    auto addBaseVertex = [&](int idx, double x, double y, double z) {
        const Vec3 v(x, y, z);
        V.row(idx) = v.normalized().transpose();
    };

    addBaseVertex(0, -1, -1, -1);
    addBaseVertex(1, 1, -1, -1);
    addBaseVertex(2, 1, 1, -1);
    addBaseVertex(3, -1, 1, -1);
    addBaseVertex(4, -1, -1, 1);
    addBaseVertex(5, 1, -1, 1);
    addBaseVertex(6, 1, 1, 1);
    addBaseVertex(7, -1, 1, 1);

    const int initialFaceCapacity = 12 * intPow4(subdivisions);
    MatX3i F(initialFaceCapacity, 3);
    int faceCount = 0;

    auto addFace = [&](int a, int b, int c) {
        F.row(faceCount++) << a, b, c;
    };

    addFace(4, 5, 6); addFace(4, 6, 7);
    addFace(1, 0, 3); addFace(1, 3, 2);
    addFace(0, 4, 7); addFace(0, 7, 3);
    addFace(5, 1, 2); addFace(5, 2, 6);
    addFace(7, 6, 2); addFace(7, 2, 3);
    addFace(0, 1, 5); addFace(0, 5, 4);

    for (int sub = 0; sub < subdivisions; ++sub) {
        std::map<int64_t, Edge> edges;
        std::vector<std::vector<int>> adj(V.rows());

        for (int fi = 0; fi < faceCount; ++fi) {
            const int a = F(fi, 0);
            const int b = F(fi, 1);
            const int c = F(fi, 2);

            addEdgeOpposite(edges, a, b, c);
            addEdgeOpposite(edges, b, c, a);
            addEdgeOpposite(edges, c, a, b);

            auto addUnique = [](std::vector<int>& list, int v) {
                if (std::find(list.begin(), list.end(), v) == list.end()) list.push_back(v);
            };
            addUnique(adj[a], b); addUnique(adj[a], c);
            addUnique(adj[b], a); addUnique(adj[b], c);
            addUnique(adj[c], a); addUnique(adj[c], b);
        }

        MatX3d nextV(V.rows() + static_cast<int>(edges.size()), 3);

        for (int i = 0; i < V.rows(); ++i) {
            const int n = static_cast<int>(adj[i].size());
            double beta = 3.0 / 16.0;
            if (n > 3) {
                const double angle = 2.0 * std::numbers::pi / static_cast<double>(n);
                beta = (1.0 / n) * (5.0 / 8.0 - std::pow(3.0 / 8.0 + 0.25 * std::cos(angle), 2));
            }

            Vec3 sumAdj = Vec3::Zero();
            for (int nb : adj[i]) sumAdj += V.row(nb).transpose();

            const Vec3 vi = V.row(i).transpose();
            const Vec3 newVi = (1.0 - n * beta) * vi + beta * sumAdj;
            nextV.row(i) = newVi.transpose();
        }

        int newVid = V.rows();
        for (auto& kv : edges) {
            Edge& e = kv.second;
            const Vec3 v1 = V.row(e.v1).transpose();
            const Vec3 v2 = V.row(e.v2).transpose();

            if (e.opposite2 != -1) {
                const Vec3 v3 = V.row(e.opposite1).transpose();
                const Vec3 v4 = V.row(e.opposite2).transpose();
                const Vec3 newV = (3.0 / 8.0) * (v1 + v2) + (1.0 / 8.0) * (v3 + v4);
                nextV.row(newVid) = newV.transpose();
            } else {
                nextV.row(newVid) = (0.5 * (v1 + v2)).transpose();
            }
            e.new_vid = newVid++;
        }

        MatX3i nextF(faceCount * 4, 3);
        int nextFaceCount = 0;
        for (int fi = 0; fi < faceCount; ++fi) {
            const int a = F(fi, 0), b = F(fi, 1), c = F(fi, 2);
            const int e1 = edges[getEdgeKey(a, b)].new_vid;
            const int e2 = edges[getEdgeKey(b, c)].new_vid;
            const int e3 = edges[getEdgeKey(c, a)].new_vid;

            nextF.row(nextFaceCount++) << a, e1, e3;
            nextF.row(nextFaceCount++) << b, e2, e1;
            nextF.row(nextFaceCount++) << c, e3, e2;
            nextF.row(nextFaceCount++) << e1, e2, e3;
        }

        V = nextV;
        F = nextF.topRows(nextFaceCount);
        faceCount = nextFaceCount;
    }

    for (int i = 0; i < V.rows(); ++i) {
        V.row(i) = (V.row(i).normalized() * radius);
    }

    auto tpl = std::shared_ptr<DropletTemplate>(new DropletTemplate());
    tpl->restV_ = V;
    tpl->F_ = F.topRows(faceCount);
    tpl->boundaryLoop_.clear();
    return tpl;
}

} // namespace wd
