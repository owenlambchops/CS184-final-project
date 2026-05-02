#include "wd/sim/operators.h"
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
    Weights w;
    const double eps = 1e-5;

    for (int fi = 0; fi < faces.rows(); ++fi) {
        const int i0 = faces(fi, 0);
        const int i1 = faces(fi, 1);
        const int i2 = faces(fi, 2);
        const Vec3 p0 = x.row(i0).transpose();
        const Vec3 p1 = x.row(i1).transpose();
        const Vec3 p2 = x.row(i2).transpose();

        // Edge vectors from each vertex
        Vec3 e0_1 = p1 - p0,  e0_2 = p2 - p0;   // edges at vertex 0
        Vec3 e1_2 = p2 - p1,  e1_0 = p0 - p1;   // edges at vertex 1
        Vec3 e2_0 = p0 - p2,  e2_1 = p1 - p2;   // edges at vertex 2

        // Cross products (their norms equal twice the sub-triangle area)
        const Vec3 cr0 = e0_1.cross(e0_2);
        const Vec3 cr1 = e1_2.cross(e1_0);
        const Vec3 cr2 = e2_0.cross(e2_1);

        double n0 = cr0.norm();
        double n1 = cr1.norm();
        double n2 = cr2.norm();

        // Guard against degenerate (zero-area) faces — mirrors np.where(norm==0, 1e-5, norm)
        if (n0 == 0.0) n0 = eps;
        if (n1 == 0.0) n1 = eps;
        if (n2 == 0.0) n2 = eps;

        // cot(angle) = dot(edges) / |cross(edges)|
        const double cot0 = e0_1.dot(e0_2) / n0;
        const double cot1 = e1_2.dot(e1_0) / n1;
        const double cot2 = e2_0.dot(e2_1) / n2;

        // Assign weights; clamp negatives to 0 (matches Python's max(0.0, ...))
        // w0 is the weight on edge (i0,i1), opposite to vertex i2 → uses cot2
        const double w0 = std::max(0.0, cot2 * 0.5);
        const double w1 = std::max(0.0, cot0 * 0.5);
        const double w2 = std::max(0.0, cot1 * 0.5);

        w[i0][i1] += w0; w[i1][i0] += w0;
        w[i1][i2] += w1; w[i2][i1] += w1;
        w[i2][i0] += w2; w[i0][i2] += w2;
    }

    return w;
}

MatX3d computeLaplacian(const MatX3d& src, const std::vector<std::vector<int>>& neighbours, const Weights& w) {
    MatX3d lap = MatX3d::Zero(src.rows(), 3);
    for (int i = 0; i < src.rows(); ++i) {
        Vec3 accum = Vec3::Zero();
        double sumW = 0.0;

        auto wi = w.find(i);
        if (wi == w.end()) continue;

        for (int j : neighbours[i]) {
            double wij = 1.0;
            auto wj = wi->second.find(j);
            if (wj != wi->second.end()) wij = wj->second;

            accum += wij * (src.row(j).transpose() - src.row(i).transpose());
            sumW += wij;
        }

        if (sumW > 0.0) {
            lap.row(i) = (accum / sumW).transpose();
        }
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
    (void)pushoutEps;
    (void)adhesionDist;
    (void)dt;

    auto& X = drop.positions();
    auto& U = drop.velocities();

    for (int i = 0; i < X.rows(); ++i) {
        const SurfaceSample s = surface.closestSample(X.row(i).transpose());
        if (s.signedDistance < 0.0) {
            // Project position onto the surface.
            X.row(i) = s.position.transpose();

            // Zero normal velocity component (keep tangential motion only).
            Vec3 v = U.row(i).transpose();
            const double vn = v.dot(s.normal);
            v -= vn * s.normal;
            U.row(i) = v.transpose();
        }
    }
}

void ViscosityOperator::apply(Droplet& drop, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();

    const double mu = std::max(0.0, drop.material().viscousDamping);
    const double eta = std::max(0.0, drop.material().laplacianViscosity);
    const double dampingGain = 1.0;

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
    for (int i = 0; i < U.rows(); ++i) {
        const Vec3 dxi = deltaX.row(i).transpose();
        if (!std::isfinite(dxi.x()) || !std::isfinite(dxi.y()) || !std::isfinite(dxi.z())) continue;

        // Surface-tension force: f_st = gamma * delta_x[i]
        const Vec3 f_st = gamma * dxi;
        // Acceleration from surface tension: a_st = f_st / density
        const Vec3 a_st = f_st / density;

        U.row(i) += (a_st * dt).transpose();
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

void EdgeLengthRegularizer::apply(
        Droplet& drop, double dt, double targetRatio, double stiffness, double maxRelativeSpeed) const {
    if (dt <= 0.0) return;
    if (stiffness <= 0.0) return;

    const auto& E = drop.edges();
    if (E.empty()) return;

    auto& X = drop.positions();
    auto& U = drop.velocities();
    const int n = X.rows();
    if (n == 0) return;

    const double restMean = std::max(drop.derived().restMeanEdgeLength, 1e-8);
    const double targetLen = std::max(1e-8, targetRatio * restMean);
    const double maxCorrSpeed = std::max(0.0, maxRelativeSpeed) * targetLen;

    MatX3d dU = MatX3d::Zero(n, 3);
    for (const auto& e : E) {
        const int i = e.x();
        const int j = e.y();
        if (i < 0 || j < 0 || i >= n || j >= n) continue;

        const Vec3 xi = X.row(i).transpose();
        const Vec3 xj = X.row(j).transpose();
        const Vec3 edge = xj - xi;
        const double len = edge.norm();
        if (len <= 1e-12) continue;

        const Vec3 dir = edge / len;
        double corrSpeed = stiffness * (len - targetLen);
        corrSpeed = std::clamp(corrSpeed, -maxCorrSpeed, maxCorrSpeed);
        const Vec3 dv = 0.5 * corrSpeed * dir;

        dU.row(i) += dv.transpose();
        dU.row(j) -= dv.transpose();
    }

    U += dt * dU;
}


} // namespace wd
