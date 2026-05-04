#include "wd/sim/operators.h"
#include "wd/sim/cgl_math.h"
#include <algorithm>
#include <cmath>

namespace wd {
namespace {

// Small numeric tolerance used for robust normalization/angle logic.
constexpr double kEps = 1e-10;
constexpr double kPi = 3.14159265358979323846;

Weights computeCotangentWeights(const MatX3d& x, const MatX3i& faces) {
    return computeCotangentWeightsCgl(toVec3List(x), toFaceList(faces));
}

// Applies the cotangent Laplacian to a per-vertex quantity (positions or velocities).
// The cgl helper fills both deltaX and deltaV; here we use deltaX as the generic output.
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

// Lumped area mass model: each triangle contributes one third of its area to each corner.
// Used by volume correction as an area-weighted measure per vertex.
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

} // namespace

// Applies external acceleration field samples directly to velocity (semi-implicit update step).
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

// Enforces surface contact by projection and velocity correction.
// - Projects near-surface/penetrating vertices to the surface offset by pushout epsilon.
// - Removes normal velocity component (non-penetration constraint).
// - Applies tangential speed reduction via simple friction thresholding.
void CollisionProjector::apply(
        Droplet& drop,
        const ISurface& surface,
        double pushoutEps,
        double adhesionDist,
        double dt) const {

    auto& X = drop.positions();
    auto& U = drop.velocities();
    const double eps = std::max(0.0, surface.material().friction);

    for (int i = 0; i < X.rows(); ++i) {
        SurfaceSample s = surface.closestSample(X.row(i).transpose());
        if (s.signedDistance < adhesionDist) {
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
            } else {
                vNew -= eps * (vNew / speed);
            }

            // 4) Immediate position correction by velocity change.
            X.row(i) += ((vNew - vOld) * dt).transpose();
            U.row(i) = vNew.transpose();
        }
    }
}

// Velocity-space smoothing/damping operator:
// - Linear damping by mu.
// - Laplacian viscosity by eta.
// - Additional nonlinear damping to suppress high-speed jitter.
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

// Surface tension via curvature flow (discrete mean-curvature force).
//
// Key idea:
// For a triangulated surface, the cotangent Laplacian of position at vertex i
// approximates mean-curvature normal: deltaX_i ~= H_i * n_i (up to scaling convention).
// Multiplying by surface tension coefficient gamma gives a force-like term that
// drives area minimization (smoothing the interface).
//
// Intuition on push/pull:
// - At locally convex "bulging out" regions, the curvature-normal term points
//   inward, so vertices are pulled inward.
// - At locally concave/indented regions, the term points outward, so vertices
//   are pushed outward.
// This bidirectional behavior reduces curvature variation and shrinks high-curvature
// features, which is exactly the capillary smoothing effect of surface tension.
//
// Integration in this code:
// f_st = gamma * deltaX, a_st = f_st / density, then v += a_st * dt.
void CurvatureFlowOperator::apply(Droplet& drop, const ISurface&, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();
    const auto& N = drop.derived().vertexNormals;
    if (X.rows() == 0 || dt <= 0.0) return;

    const auto neighbours = buildNeighbours(drop.faces(), X.rows());
    const Weights w = computeCotangentWeights(X, drop.faces());
    const MatX3d lapX = computeLaplacian(X, neighbours, w);

    const double scale = (drop.material().surfaceTension / std::max(drop.material().density, 1e-8)) * dt;
    U += scale * lapX;

    // Small tangential redistribution to one-ring neighbors:
    // closer neighbors get stronger effect via inverse edge-length weighting.
    constexpr double kTangentialNeighborGain = 0.12;
    for (int i = 0; i < X.rows(); ++i) {
        Vec3 ni = N.row(i).transpose();
        const double nn = ni.norm();
        if (nn <= 1e-12) continue;
        ni /= nn;

        const Vec3 li = lapX.row(i).transpose();
        const double lapMag = li.norm();
        if (!std::isfinite(lapMag) || lapMag <= 1e-12) continue;
        const double curvatureSign = li.dot(ni) >= 0.0 ? 1.0 : -1.0;

        for (int j : neighbours[static_cast<size_t>(i)]) {
            if (j < 0 || j >= U.rows() || j == i) continue;

            Vec3 edge = X.row(j).transpose() - X.row(i).transpose();
            const double edgeLen = edge.norm();
            if (edgeLen <= 1e-8) continue;

            Vec3 tangentDir = edge - edge.dot(ni) * ni;
            const double tn = tangentDir.norm();
            if (tn <= 1e-12) continue;
            tangentDir /= tn;

            const double weight = 1.0 / edgeLen; // stronger when closer
            const Vec3 dv =
                0.5 * (curvatureSign * kTangentialNeighborGain * std::abs(scale) * lapMag * weight) * tangentDir;
            U.row(j) += dv.transpose();
        }
    }
}

// Contact-angle hysteresis operator near the contact line.
// Applies restoring force only when contact angle exits [receding, advancing] band.
void ContactLineOperator::apply(Droplet& drop, const ISurface& surface, double dt, double adhesionDist) const {
    auto& U = drop.velocities();
    const auto& X = drop.positions();
    const auto& N = drop.derived().vertexNormals;

    const SurfaceMaterialParams& surfaceMaterial = surface.material();
    const double alpha = surfaceMaterial.contactStiffness;
    const double receding = surfaceMaterial.recContactAngleDeg * kPi / 180.0;
    const double advancing = surfaceMaterial.advContactAngleDeg * kPi / 180.0;
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

// Mesh-quality regularization in edge space.
// Short edges repel, long edges attract, then result is projected to tangent plane.
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

void ContactBandEdgeProjector::apply(
        Droplet& drop,
        const ISurface& surface,
        double adhesionDist,
        double targetRatio,
        int iterations,
        double relaxation) const {
    auto& X = drop.positions();
    const auto& E = drop.edges();
    const int n = X.rows();
    if (n == 0 || E.empty()) return;

    const double safeAdhesion = std::max(0.0, adhesionDist);
    const int iters = std::max(0, iterations);
    const double omega = std::clamp(relaxation, 0.0, 1.0);
    if (iters == 0 || omega <= 0.0) return;

    const double meanEdge = std::max(drop.derived().meanEdgeLength, 1e-8);
    const double targetLen = std::max(1e-8, targetRatio * meanEdge);

    for (int it = 0; it < iters; ++it) {
        drop.updateDerived();

        std::vector<bool> inBand(static_cast<size_t>(n), false);
        std::vector<Vec3> bandNormal(static_cast<size_t>(n), Vec3::UnitY());
        for (int i = 0; i < n; ++i) {
            SurfaceSample s = surface.closestSample(X.row(i).transpose());
            if (s.signedDistance >= 0.0 && s.signedDistance <= safeAdhesion) {
                inBand[static_cast<size_t>(i)] = true;
                bandNormal[static_cast<size_t>(i)] = s.normal.normalized();
            }
        }

        for (const auto& e : E) {
            const int i = e.x();
            const int j = e.y();
            if (i < 0 || j < 0 || i >= n || j >= n) continue;
            if (!inBand[static_cast<size_t>(i)] || !inBand[static_cast<size_t>(j)]) continue;

            const Vec3 xi = X.row(i).transpose();
            const Vec3 xj = X.row(j).transpose();
            const Vec3 d = xj - xi;
            const double len = d.norm();
            if (len <= 1e-12) continue;

            // PBD-style symmetric correction: reduce edge-length error without high-force injection.
            const double C = len - targetLen;
            const Vec3 dir = d / len;
            Vec3 corr = 0.5 * omega * C * dir;

            Vec3 ni = bandNormal[static_cast<size_t>(i)];
            Vec3 nj = bandNormal[static_cast<size_t>(j)];
            corr -= corr.dot(ni) * ni; // tangent at i
            Vec3 corrJ = corr - corr.dot(nj) * nj; // tangent at j

            X.row(i) += corr.transpose();
            X.row(j) -= corrJ.transpose();
        }
    }
}

// Public wrapper for closed-volume estimate.
double VolumeCorrector::computeClosedVolume(const Droplet& drop, const ISurface&) const {
    return computeClosedVolumeLocal(drop);
}

// Two-stage volume stabilization:
// 1) Local velocity correction (reduce volume-changing deformation modes).
// 2) Global positional correction to hit target volume exactly.
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
