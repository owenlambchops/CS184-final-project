#include "wd/sim/operators.h"
#include "wd/sim/cgl_math.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace wd {
namespace {

constexpr double kEps = 1e-10;
constexpr double kPi = 3.14159265358979323846;

std::vector<std::vector<int>> buildNeighbours(const MatX3i& faces, int n) {
    std::vector<std::set<int>> adjSets(n);
    for (int i = 0; i < faces.rows(); ++i) {
        const int a = faces(i, 0);
        const int b = faces(i, 1);
        const int c = faces(i, 2);
        adjSets[a].insert(b); adjSets[a].insert(c);
        adjSets[b].insert(a); adjSets[b].insert(c);
        adjSets[c].insert(a); adjSets[c].insert(b);
    }

    std::vector<std::vector<int>> neighbours(n);
    for (int i = 0; i < n; ++i) {
        neighbours[i] = std::vector<int>(adjSets[i].begin(), adjSets[i].end());
    }
    return neighbours;
}

Weights computeCotangentWeights(const MatX3d& x, const MatX3i& faces) {
    return computeCotangentWeightsCgl(toVec3List(x), toFaceList(faces));
}

MatX3d computeLaplacian(const MatX3d& src, const std::vector<std::vector<int>>& neighbours, const Weights& w) {
    std::vector<Vec3> srcList = toVec3List(src);
    std::vector<Vec3> deltaX;
    std::vector<Vec3> deltaV;
    computeLaplaciansCgl(srcList, srcList, neighbours, w, deltaX, deltaV);

    MatX3d lap = MatX3d::Zero(src.rows(), 3);
    for (int i = 0; i < src.rows(); ++i) {
        lap.row(i) = deltaX[static_cast<size_t>(i)].transpose();
    }
    return lap;
}

double computeClosedVolumeLocal(const Droplet& drop) {
    const auto& X = drop.positions();
    const auto& F = drop.faces();
    if (X.rows() == 0 || F.rows() == 0) return 0.0;

    const Vec3 com = X.colwise().mean().transpose();
    double V = 0.0;
    for (int i = 0; i < F.rows(); ++i) {
        Vec3 x0 = X.row(F(i, 0)).transpose();
        Vec3 x1 = X.row(F(i, 1)).transpose();
        Vec3 x2 = X.row(F(i, 2)).transpose();
        V += (x0 - com).dot((x1 - com).cross(x2 - com)) / 6.0;
    }
    return std::abs(V);
}

std::vector<double> computeLumpedAreas(const Droplet& drop) {
    const auto& X = drop.positions();
    const auto& F = drop.faces();
    std::vector<double> areas(X.rows(), 0.0);

    for (int fi = 0; fi < F.rows(); ++fi) {
        const int i0 = F(fi, 0);
        const int i1 = F(fi, 1);
        const int i2 = F(fi, 2);
        const Vec3 p0 = X.row(i0).transpose();
        const Vec3 p1 = X.row(i1).transpose();
        const Vec3 p2 = X.row(i2).transpose();
        const double triArea = 0.5 * ((p1 - p0).cross(p2 - p0)).norm();
        const double share = triArea / 3.0;
        areas[i0] += share;
        areas[i1] += share;
        areas[i2] += share;
    }
    return areas;
}

double computeMeanEdgeLength(const Droplet& drop) {
    const auto& X = drop.positions();
    const auto& F = drop.faces();
    double sumLen = 0.0;
    int edgeCount = 0;

    for (int fi = 0; fi < F.rows(); ++fi) {
        const int i0 = F(fi, 0);
        const int i1 = F(fi, 1);
        const int i2 = F(fi, 2);

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

bool faceContainsEdge(const Eigen::Vector3i& f, int a, int b) {
    int hits = 0;
    for (int k = 0; k < 3; ++k) {
        if (f[k] == a || f[k] == b) ++hits;
    }
    return hits == 2;
}

bool replaceVertexInFace(Eigen::Vector3i& f, int oldV, int newV) {
    bool changed = false;
    for (int k = 0; k < 3; ++k) {
        if (f[k] == oldV) {
            f[k] = newV;
            changed = true;
        }
    }
    return changed;
}

bool isDegenerateFace(const Eigen::Vector3i& f) {
    return (f[0] == f[1]) || (f[1] == f[2]) || (f[2] == f[0]);
}

void compactMeshData(Droplet& drop) {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    auto& F = drop.faces();

    std::vector<Eigen::Vector3i> keptFaces;
    keptFaces.reserve(static_cast<size_t>(F.rows()));
    for (int i = 0; i < F.rows(); ++i) {
        const Eigen::Vector3i fi = F.row(i);
        if (!isDegenerateFace(fi)) keptFaces.push_back(fi);
    }

    std::vector<char> used(static_cast<size_t>(X.rows()), 0);
    for (const auto& f : keptFaces) {
        used[static_cast<size_t>(f[0])] = 1;
        used[static_cast<size_t>(f[1])] = 1;
        used[static_cast<size_t>(f[2])] = 1;
    }

    std::vector<int> map(static_cast<size_t>(X.rows()), -1);
    int newCount = 0;
    for (int i = 0; i < X.rows(); ++i) {
        if (used[static_cast<size_t>(i)]) map[static_cast<size_t>(i)] = newCount++;
    }

    MatX3d Xn = MatX3d::Zero(newCount, 3);
    MatX3d Un = MatX3d::Zero(newCount, 3);
    for (int i = 0; i < X.rows(); ++i) {
        const int ni = map[static_cast<size_t>(i)];
        if (ni < 0) continue;
        Xn.row(ni) = X.row(i);
        Un.row(ni) = U.row(i);
    }

    MatX3i Fn(static_cast<int>(keptFaces.size()), 3);
    for (int i = 0; i < static_cast<int>(keptFaces.size()); ++i) {
        Fn(i, 0) = map[static_cast<size_t>(keptFaces[static_cast<size_t>(i)][0])];
        Fn(i, 1) = map[static_cast<size_t>(keptFaces[static_cast<size_t>(i)][1])];
        Fn(i, 2) = map[static_cast<size_t>(keptFaces[static_cast<size_t>(i)][2])];
    }

    X = std::move(Xn);
    U = std::move(Un);
    F = std::move(Fn);
    drop.edges().clear();
}

bool splitLongestEdge(Droplet& drop, double splitLen) {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    auto& F = drop.faces();
    const auto& E = drop.edges();
    if (E.empty()) return false;

    int bestI = -1;
    int bestJ = -1;
    double bestLen = splitLen;
    for (const auto& e : E) {
        const int i = e.x();
        const int j = e.y();
        if (i < 0 || j < 0 || i >= X.rows() || j >= X.rows()) continue;
        const double len = (X.row(i).transpose() - X.row(j).transpose()).norm();
        if (len > bestLen) {
            bestLen = len;
            bestI = i;
            bestJ = j;
        }
    }
    if (bestI < 0 || bestJ < 0) return false;

    const int n = X.rows();
    MatX3d Xn(n + 1, 3);
    MatX3d Un(n + 1, 3);
    Xn.topRows(n) = X;
    Un.topRows(n) = U;
    Xn.row(n) = 0.5 * (X.row(bestI) + X.row(bestJ));
    Un.row(n) = 0.5 * (U.row(bestI) + U.row(bestJ));
    X = std::move(Xn);
    U = std::move(Un);

    std::vector<Eigen::Vector3i> facesOut;
    facesOut.reserve(static_cast<size_t>(F.rows()) + 8);
    for (int fi = 0; fi < F.rows(); ++fi) {
        Eigen::Vector3i f = F.row(fi);
        if (!faceContainsEdge(f, bestI, bestJ)) {
            facesOut.push_back(f);
            continue;
        }

        int k = -1;
        for (int t = 0; t < 3; ++t) {
            if (f[t] != bestI && f[t] != bestJ) {
                k = f[t];
                break;
            }
        }
        if (k < 0) continue;
        facesOut.emplace_back(bestI, n, k);
        facesOut.emplace_back(n, bestJ, k);
    }

    MatX3i Fn(static_cast<int>(facesOut.size()), 3);
    for (int i = 0; i < static_cast<int>(facesOut.size()); ++i) Fn.row(i) = facesOut[static_cast<size_t>(i)];
    F = std::move(Fn);
    drop.edges().clear();
    return true;
}

bool collapseShortestEdge(Droplet& drop, double collapseLen) {
    auto& X = drop.positions();
    auto& U = drop.velocities();
    auto& F = drop.faces();
    const auto& E = drop.edges();
    if (E.empty()) return false;

    int bestI = -1;
    int bestJ = -1;
    double bestLen = collapseLen;
    for (const auto& e : E) {
        const int i = e.x();
        const int j = e.y();
        if (i < 0 || j < 0 || i >= X.rows() || j >= X.rows()) continue;
        const double len = (X.row(i).transpose() - X.row(j).transpose()).norm();
        if (len < bestLen) {
            bestLen = len;
            bestI = i;
            bestJ = j;
        }
    }
    if (bestI < 0 || bestJ < 0) return false;
    if (bestI == bestJ) return false;
    if (bestI > bestJ) std::swap(bestI, bestJ);

    X.row(bestI) = 0.5 * (X.row(bestI) + X.row(bestJ));
    U.row(bestI) = 0.5 * (U.row(bestI) + U.row(bestJ));

    for (int fi = 0; fi < F.rows(); ++fi) {
        Eigen::Vector3i f = F.row(fi);
        if (replaceVertexInFace(f, bestJ, bestI)) {
            F.row(fi) = f;
        }
    }

    compactMeshData(drop);
    return true;
}

} // namespace

void ExternalForceOperator::apply(
        Droplet& drop,
        const ISurface&,
        const IForceField& field,
        double timeSec,
        double dt) const {
    auto& X = drop.positions();
    auto& U = drop.velocities();

    for (int i = 0; i < X.rows(); ++i) {
        const Vec3 a_ext = field.sample(X.row(i).transpose(), timeSec);
        U.row(i) += (a_ext * dt).transpose();
    }
}

void CollisionProjector::apply(
        Droplet& drop,
        const ISurface& surface,
        double pushoutEps,
        double adhesionDist,
        double dt) const {
    (void)adhesionDist;

    auto& X = drop.positions();
    auto& U = drop.velocities();
    const double eps = std::max(0.0, drop.material().friction);

    for (int i = 0; i < X.rows(); ++i) {
        SurfaceSample s = surface.closestSample(X.row(i).transpose());
        if (s.signedDistance < 0.0) {
            // 1) Project penetrated vertex to the closest point on the solid.
            X.row(i) = (s.position + pushoutEps * s.normal.normalized()).transpose();
            s = surface.closestSample(X.row(i).transpose());

            const Vec3 n = s.normal.normalized();
            const Vec3 vOld = U.row(i).transpose();

            // 2) Eq. (1): remove relative normal velocity (v_s = 0 for static solid).
            Vec3 vNew = vOld - (vOld.dot(n)) * n;

            // 3) Eq. (2): friction magnitude rule on colliding vertices.
            const double speed = vNew.norm();
            if (speed < eps) {
                vNew.setZero();
            } else if (speed > 1e-12) {
                vNew -= eps * (vNew / speed);
            }

            // 4) Immediate position correction by velocity change.
            X.row(i) += ((vNew - vOld) * dt).transpose();
            U.row(i) = vNew.transpose();
        }
    }
}

void ViscosityOperator::apply(Droplet& drop, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();

    const double mu = std::max(0.0, drop.material().viscousDamping);
    double eta = std::max(0.0, drop.material().laplacianViscosity);
    const double dampingGain = 1.0;
    const double minEdge = std::max(drop.derived().minEdgeLength, 1e-8);
    const double edgeSkew = drop.derived().maxEdgeLength / minEdge;
    if (edgeSkew > 1.5) eta *= 2.0; // paper-style extra viscosity under poor mesh quality

    auto neighbours = buildNeighbours(drop.faces(), X.rows());
    Weights w = computeCotangentWeights(X, drop.faces());
    MatX3d deltaV = computeLaplacian(U, neighbours, w);

    double meanSpeed = 0.0;
    for (int i = 0; i < U.rows(); ++i) {
        const Vec3 vi = U.row(i).transpose();
        const Vec3 dvi = deltaV.row(i).transpose();

        Vec3 vDamped = vi * (1.0 - mu * dt) + dvi * (eta * dt);
        const double speed = vi.norm();

        const double lin = mu * dampingGain;
        const double quad = (0.2 + 0.3 * eta) * dampingGain;
        const double damping = 1.0 / (1.0 + lin * dt + quad * speed * dt);

        U.row(i) = (vDamped * damping).transpose();
        meanSpeed += speed;
    }

    meanSpeed /= std::max(1, static_cast<int>(U.rows()));
    if (meanSpeed < 0.25) {
        const double factor = std::max(0.0, 1.0 - 0.3 * dt * dampingGain);
        U *= factor;
    }
}

void CurvatureFlowOperator::apply(Droplet& drop, const ISurface&, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();

    auto neighbours = buildNeighbours(drop.faces(), X.rows());
    Weights w = computeCotangentWeights(X, drop.faces());
    MatX3d deltaX = computeLaplacian(X, neighbours, w);

    const double gamma = drop.material().surfaceTension;
    const double density = std::max(drop.material().density, 1e-8);
    for (int i = 0; i < X.rows(); ++i) {
        const Vec3 dxi = deltaX.row(i).transpose();
        if (!std::isfinite(dxi.x()) || !std::isfinite(dxi.y()) || !std::isfinite(dxi.z())) continue;

        // Python parity:
        // f_st = gamma * delta_x, then a_st = f_st / density and v += a_st * dt.
        const Vec3 fSt = gamma * dxi;
        const Vec3 aSt = fSt / density;
        U.row(i) += (aSt * dt).transpose();
    }
}

void ContactLineOperator::apply(Droplet& drop, const ISurface& surface, double dt, double adhesionDist) const {
    auto& U = drop.velocities();
    const auto& X = drop.positions();
    const auto& N = drop.derived().vertexNormals;

    const double alpha = drop.material().contactStiffness;
    const double receding = drop.material().recContactAngleDeg * kPi / 180.0;
    const double advancing = drop.material().advContactAngleDeg * kPi / 180.0;
    const double density = std::max(drop.material().density, 1e-8);

    if (receding > advancing) return;

    for (int i = 0; i < X.rows(); ++i) {
        SurfaceSample s = surface.closestSample(X.row(i).transpose());
        if (s.signedDistance > adhesionDist) continue;

        Vec3 n_li = N.row(i).transpose();
        Vec3 n_i = s.normal.normalized();
        double dotVal = std::clamp(n_li.dot(n_i), -1.0, 1.0);
        double angle = std::acos(dotVal);

        Vec3 n_p = n_li - dotVal * n_i;
        double n_p_norm = n_p.norm();
        if (n_p_norm < kEps) continue;
        Vec3 n_p_dir = n_p / n_p_norm;

        if (receding < angle && angle < advancing) continue;

        Vec3 fBoundary = Vec3::Zero();
        if (angle <= receding) {
            fBoundary = alpha * (angle - receding) * n_p_dir;
        } else {
            fBoundary = alpha * (angle - advancing) * n_p_dir;
        }

        U.row(i) += ((fBoundary / density) * dt).transpose();
    }
}

void VertexRepulsionOperator::apply(
        Droplet& drop,
        double dt,
        double targetRatio,
        double strength,
        double maxAccel) const {
    if (dt <= 0.0) return;
    if (strength <= 0.0) return;

    const auto& E = drop.edges();
    const auto& X = drop.positions();
    const auto& N = drop.derived().vertexNormals;
    auto& U = drop.velocities();
    const int n = X.rows();
    if (n == 0 || E.empty()) return;

    const double meanEdge = std::max(drop.derived().meanEdgeLength, 1e-8);
    const double targetLen = std::max(1e-8, targetRatio * meanEdge);
    const double density = std::max(drop.material().density, 1e-8);
    const double accelCap = std::max(0.0, maxAccel);

    MatX3d aRep = MatX3d::Zero(n, 3);
    for (const auto& e : E) {
        const int i = e.x();
        const int j = e.y();
        if (i < 0 || j < 0 || i >= n || j >= n) continue;

        const Vec3 xi = X.row(i).transpose();
        const Vec3 xj = X.row(j).transpose();
        const Vec3 d = xj - xi;
        const double len = d.norm();
        if (len <= 1e-12) continue;

        const Vec3 dir = d / len;
        const double rel = (len - targetLen) / targetLen;
        const double mag = strength * rel * std::abs(rel);
        if (std::abs(mag) <= 1e-12) continue;

        // Short edge (rel < 0): pushes vertices apart.
        // Long edge  (rel > 0): pulls vertices together.
        aRep.row(i) += ( mag * dir / density).transpose();
        aRep.row(j) += (-mag * dir / density).transpose();
    }

    for (int i = 0; i < n; ++i) {
        Vec3 ai = aRep.row(i).transpose();
        Vec3 ni = N.row(i).transpose();
        const double nn = ni.norm();
        if (nn > 1e-12) {
            ni /= nn;
            ai -= ai.dot(ni) * ni; // tangent projection to reduce shape-volume side effects
        }
        const double an = ai.norm();
        if (accelCap > 0.0 && an > accelCap) ai *= accelCap / std::max(an, 1e-8);
        U.row(i) += (ai * dt).transpose();
    }
}

void AdaptiveRemesher::apply(
        Droplet& drop,
        double targetLen,
        double splitThresh,
        double collapseThresh,
        int maxOps) const {
    if (maxOps <= 0) return;
    if (drop.positions().rows() < 4 || drop.faces().rows() < 4) return;

    const double baseLen = (targetLen > 1e-8) ? targetLen : std::max(computeMeanEdgeLength(drop), 1e-8);
    const double splitLen = std::max(baseLen * splitThresh, 1e-8);
    const double collapseLen = std::max(baseLen * collapseThresh, 1e-8);
    if (collapseLen >= splitLen) return;

    for (int op = 0; op < maxOps; ++op) {
        drop.updateDerived();
        bool changed = splitLongestEdge(drop, splitLen);
        if (!changed) changed = collapseShortestEdge(drop, collapseLen);
        if (!changed) break;
        compactMeshData(drop);
    }
}

double VolumeCorrector::computeClosedVolume(const Droplet& drop, const ISurface&) const {
    return computeClosedVolumeLocal(drop);
}

void VolumeCorrector::apply(Droplet& drop, const ISurface& surface, double dt) const {
    (void)surface;
    (void)dt;

    auto& X = drop.positions();
    auto& U = drop.velocities();
    const auto& N = drop.derived().vertexNormals;
    const auto& F = drop.faces();
    const int n = X.rows();
    if (n == 0) return;
    const bool doLocal = drop.material().enableLocalVolumeCorrection;
    const bool doGlobal = drop.material().enableGlobalVolumeCorrection;
    if (!doLocal && !doGlobal) return;

    const double target = (drop.targetVolume() > 0.0) ? drop.targetVolume() : drop.derived().restVolume;
    if (target <= 0.0) return;

    const std::vector<double> lumpedAreas = computeLumpedAreas(drop);
    double sumArea = 0.0;
    for (double a : lumpedAreas) sumArea += a;
    if (sumArea <= 1e-12) return;

    const double current = computeClosedVolumeLocal(drop);
    const double dV = target - current;
    const double relErr = std::abs(dV) / std::max(target, 1e-8);
    if (relErr < 1e-5) return;

    if (doLocal) {
        const auto neighbours = buildNeighbours(F, n);

        // 1) Remove rigid velocity (translation + rotation): u_i = v_i - v_rigid_i.
        const Vec3 com = X.colwise().mean().transpose();
        const Vec3 vCom = U.colwise().mean().transpose();

        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        Vec3 b = Vec3::Zero();
        for (int i = 0; i < n; ++i) {
            const double wi = std::max(lumpedAreas[i], 0.0);
            const Vec3 r = X.row(i).transpose() - com;
            const Vec3 vRel = U.row(i).transpose() - vCom;
            M += wi * ((r.squaredNorm() * Eigen::Matrix3d::Identity()) - (r * r.transpose()));
            b += wi * r.cross(vRel);
        }

        Vec3 omega = Vec3::Zero();
        Eigen::LDLT<Eigen::Matrix3d> ldlt(M);
        if (ldlt.info() == Eigen::Success) {
            omega = ldlt.solve(b);
            if (!omega.array().isFinite().all()) omega.setZero();
        }

        MatX3d uDef = MatX3d::Zero(n, 3);
        std::vector<double> aRate(n, 0.0);
        for (int i = 0; i < n; ++i) {
            const Vec3 r = X.row(i).transpose() - com;
            const Vec3 vRigid = vCom + omega.cross(r);
            const Vec3 u = U.row(i).transpose() - vRigid;
            uDef.row(i) = u.transpose();

            Vec3 ni = N.row(i).transpose();
            const double niNorm = ni.norm();
            if (niNorm > 1e-12) ni /= niNorm;
            else ni.setZero();
            aRate[i] = u.dot(ni);
        }

        // 2) Eq. (10)-(11): local area-weighted average, subtract normal component from deforming velocity.
        for (int i = 0; i < n; ++i) {
            double num = lumpedAreas[i] * aRate[i];
            double den = lumpedAreas[i];
            for (int j : neighbours[i]) {
                const double Aj = lumpedAreas[j];
                num += Aj * aRate[j];
                den += Aj;
            }
            if (den <= 1e-12) continue;
            const double aBar = num / den;

            Vec3 ni = N.row(i).transpose();
            const double nn = ni.norm();
            if (nn <= 1e-12) continue;
            ni /= nn;

            const Vec3 uCorr = uDef.row(i).transpose() - aBar * ni;
            const Vec3 r = X.row(i).transpose() - com;
            const Vec3 vRigid = vCom + omega.cross(r);
            U.row(i) = (vRigid + uCorr).transpose();
        }
    }

    // Global volume correction: d = ΔV / A, then x_i += d n_i.
    if (doGlobal) {
        const double d = dV / sumArea;
        for (int i = 0; i < n; ++i) {
            Vec3 ni = N.row(i).transpose();
            const double nn = ni.norm();
            if (nn <= 1e-12) continue;
            ni /= nn;
            X.row(i) += (d * ni).transpose();
        }
    }
}

} // namespace wd
