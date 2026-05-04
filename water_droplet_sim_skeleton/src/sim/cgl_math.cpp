#include "wd/sim/cgl_math.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

// CGAL
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/boost/graph/helpers.h>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace wd {

std::vector<Vec3> toVec3List(const MatX3d& X) {
    std::vector<Vec3> out;
    out.reserve(static_cast<size_t>(X.rows()));
    for (int i = 0; i < X.rows(); ++i) {
        out.push_back(X.row(i).transpose());
    }
    return out;
}

std::vector<Face> toFaceList(const MatX3i& F) {
    std::vector<Face> out;
    out.reserve(static_cast<size_t>(F.rows()));
    for (int i = 0; i < F.rows(); ++i) {
        out.emplace_back(F(i, 0), F(i, 1), F(i, 2));
    }
    return out;
}

std::vector<std::vector<int>> buildNeighbours(const MatX3i& F, int nVertices) {
    std::vector<std::set<int>> adjSets(static_cast<size_t>(std::max(0, nVertices)));
    for (int i = 0; i < F.rows(); ++i) {
        const int a = F(i, 0);
        const int b = F(i, 1);
        const int c = F(i, 2);
        if (a >= 0 && a < nVertices && b >= 0 && b < nVertices) {
            adjSets[static_cast<size_t>(a)].insert(b);
            adjSets[static_cast<size_t>(b)].insert(a);
        }
        if (b >= 0 && b < nVertices && c >= 0 && c < nVertices) {
            adjSets[static_cast<size_t>(b)].insert(c);
            adjSets[static_cast<size_t>(c)].insert(b);
        }
        if (c >= 0 && c < nVertices && a >= 0 && a < nVertices) {
            adjSets[static_cast<size_t>(c)].insert(a);
            adjSets[static_cast<size_t>(a)].insert(c);
        }
    }

    std::vector<std::vector<int>> neighbours(static_cast<size_t>(std::max(0, nVertices)));
    for (int i = 0; i < nVertices; ++i) {
        const auto& setRef = adjSets[static_cast<size_t>(i)];
        neighbours[static_cast<size_t>(i)] = std::vector<int>(setRef.begin(), setRef.end());
    }
    return neighbours;
}

std::vector<Eigen::Vector2i> buildUniqueEdgesFromFaces(const MatX3i& F) {
    std::set<std::pair<int, int>> edgeSet;
    auto addEdge = [&edgeSet](int a, int b) {
        if (a > b) std::swap(a, b);
        edgeSet.emplace(a, b);
    };

    for (int fi = 0; fi < F.rows(); ++fi) {
        addEdge(F(fi, 0), F(fi, 1));
        addEdge(F(fi, 1), F(fi, 2));
        addEdge(F(fi, 2), F(fi, 0));
    }

    std::vector<Eigen::Vector2i> edges;
    edges.reserve(edgeSet.size());
    for (const auto& edge : edgeSet) {
        edges.emplace_back(edge.first, edge.second);
    }
    return edges;
}

double computeMeanEdgeLengthFromFaces(const MatX3d& X, const MatX3i& F) {
    double sumLen = 0.0;
    int edgeCount = 0;
    for (int fi = 0; fi < F.rows(); ++fi) {
        const int i0 = F(fi, 0);
        const int i1 = F(fi, 1);
        const int i2 = F(fi, 2);
        if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= X.rows() || i1 >= X.rows() || i2 >= X.rows()) continue;
        const Vec3 p0 = X.row(i0).transpose();
        const Vec3 p1 = X.row(i1).transpose();
        const Vec3 p2 = X.row(i2).transpose();
        sumLen += (p1 - p0).norm();
        sumLen += (p2 - p1).norm();
        sumLen += (p0 - p2).norm();
        edgeCount += 3;
    }
    if (edgeCount == 0) return 0.0;
    return sumLen / static_cast<double>(edgeCount);
}

Weights computeCotangentWeightsCgl(const std::vector<Vec3>& x, const std::vector<Face>& faces) {
    Weights w;
    for (const auto& f : faces) {
        const int i0 = f[0];
        const int i1 = f[1];
        const int i2 = f[2];
        const Vec3 v0 = x[i0];
        const Vec3 v1 = x[i1];
        const Vec3 v2 = x[i2];

        auto getCot = [](const Vec3& a, const Vec3& b) {
            const Vec3 c = a.cross(b);
            const double n = c.norm();
            if (n < 1e-10) return 0.0;
            return a.dot(b) / n;
        };

        const double cot0 = getCot(v1 - v0, v2 - v0);
        const double cot1 = getCot(v2 - v1, v0 - v1);
        const double cot2 = getCot(v0 - v2, v1 - v2);

        const double w0 = std::max(0.0, cot2 * 0.5);
        const double w1 = std::max(0.0, cot0 * 0.5);
        const double w2 = std::max(0.0, cot1 * 0.5);

        w[i0][i1] += w0;
        w[i1][i0] += w0;
        w[i1][i2] += w1;
        w[i2][i1] += w1;
        w[i2][i0] += w2;
        w[i0][i2] += w2;
    }
    return w;
}

std::pair<std::vector<Vec3>, double> computeVertexNormalsAndVolumeCgl(
        const std::vector<Vec3>& x,
        const std::vector<Face>& faces) {
    std::vector<Vec3> normals(x.size(), Vec3::Zero());
    double volume = 0.0;
    Vec3 com = Vec3::Zero();

    if (x.empty()) return {normals, 0.0};

    for (const auto& v : x) {
        com += v;
    }
    com /= static_cast<double>(x.size());

    for (const auto& f : faces) {
        const int i0 = f[0];
        const int i1 = f[1];
        const int i2 = f[2];
        const Vec3 x0 = x[i0];
        const Vec3 x1 = x[i1];
        const Vec3 x2 = x[i2];
        Vec3 nFace = (x1 - x0).cross(x2 - x0);

        if (nFace.dot(x0 - com) < 0.0) {
            nFace *= -1.0;
        }

        normals[i0] += nFace;
        normals[i1] += nFace;
        normals[i2] += nFace;

        volume += (x0 - com).dot((x1 - com).cross(x2 - com)) / 6.0;
    }

    for (auto& n : normals) {
        const double nn = n.norm();
        if (nn > 1e-12) n /= nn;
        else n = Vec3::UnitY();
    }

    return {normals, std::abs(volume)};
}

void computeLaplaciansCgl(
        const std::vector<Vec3>& x,
        const std::vector<Vec3>& v,
        const std::vector<std::vector<int>>& neighbours,
        const Weights& w,
        std::vector<Vec3>& deltaX,
        std::vector<Vec3>& deltaV) {
    deltaX.assign(x.size(), Vec3::Zero());
    deltaV.assign(v.size(), Vec3::Zero());

    for (int i = 0; i < static_cast<int>(x.size()); ++i) {
        double sumW = 0.0;
        const auto itI = w.find(i);
        if (itI == w.end()) continue;

        for (int j : neighbours[i]) {
            double wij = 1.0;
            const auto itJ = itI->second.find(j);
            if (itJ != itI->second.end()) wij = itJ->second;

            deltaX[i] += (x[j] - x[i]) * wij;
            deltaV[i] += (v[j] - v[i]) * wij;
            sumW += wij;
        }

        if (sumW > 0.0) {
            deltaX[i] /= sumW;
            deltaV[i] /= sumW;
        }
    }
}

} // namespace wd

namespace wd {

// Helper: convert MatX3d/MatX3i -> CGAL::Surface_mesh
static bool to_cgal_mesh(const wd::MatX3d& X, const wd::MatX3i& F, CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& out) {
    using Kernel = CGAL::Simple_cartesian<double>;
    using Point = Kernel::Point_3;
    out.clear();
    try {
        std::vector<CGAL::Surface_mesh<Point>::Vertex_index> verts;
        verts.reserve(static_cast<size_t>(X.rows()));
        for (int i = 0; i < X.rows(); ++i) {
            verts.push_back(out.add_vertex(Point(X(i, 0), X(i, 1), X(i, 2))));
        }
        for (int fi = 0; fi < F.rows(); ++fi) {
            const int a = F(fi, 0);
            const int b = F(fi, 1);
            const int c = F(fi, 2);
            if (a < 0 || b < 0 || c < 0) continue;
            out.add_face(verts[static_cast<size_t>(a)], verts[static_cast<size_t>(b)], verts[static_cast<size_t>(c)]);
        }
    } catch (...) {
        return false;
    }
    return true;
}

// Helper: convert CGAL::Surface_mesh -> MatX3d/MatX3i
static void from_cgal_mesh(const CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& in, wd::MatX3d& X, wd::MatX3i& F) {
    using Kernel = CGAL::Simple_cartesian<double>;
    using Point = Kernel::Point_3;
    std::vector<CGAL::Surface_mesh<Point>::Vertex_index> vmap;
    vmap.reserve(num_vertices(in));
    std::unordered_map<CGAL::Surface_mesh<Point>::Vertex_index, int> indexMap;
    int idx = 0;
    X.resize(num_vertices(in), 3);
    for (auto v : in.vertices()) {
        const Point& p = in.point(v);
        X(idx, 0) = p.x();
        X(idx, 1) = p.y();
        X(idx, 2) = p.z();
        indexMap[v] = idx;
        ++idx;
    }
    int faceCount = static_cast<int>(num_faces(in));
    F.resize(faceCount, 3);
    int fi = 0;
    for (auto f : in.faces()) {
        std::vector<int> verts;
        for (auto v : CGAL::vertices_around_face(in.halfedge(f), in)) {
            verts.push_back(indexMap[v]);
        }
        if (verts.size() >= 3) {
            F(fi, 0) = verts[0];
            F(fi, 1) = verts[1];
            F(fi, 2) = verts[2];
            ++fi;
        }
    }
    if (fi != faceCount) {
        F.conservativeResize(fi, 3);
    }
}

bool booleanUnionMeshes(
    const wd::MatX3d& A_X,
    const wd::MatX3i& A_F,
    const wd::MatX3d& B_X,
    const wd::MatX3i& B_F,
    wd::MatX3d& outX,
    wd::MatX3i& outF) {
    using Kernel = CGAL::Simple_cartesian<double>;
    using Point = Kernel::Point_3;
    using Mesh = CGAL::Surface_mesh<Point>;

    Mesh mA, mB, mOut;
    if (!to_cgal_mesh(A_X, A_F, mA)) return false;
    if (!to_cgal_mesh(B_X, B_F, mB)) return false;

    try {
        PMP::corefine_and_compute_union(mA, mB, mOut);
    } catch (...) {
        return false;
    }

    from_cgal_mesh(mOut, outX, outF);
    return true;
}

} // namespace wd
