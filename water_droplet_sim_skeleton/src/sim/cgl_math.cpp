#include "wd/sim/cgl_math.h"

#include <algorithm>
#include <cmath>

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
