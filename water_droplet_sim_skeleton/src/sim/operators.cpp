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
// Zhang et al. collision model formulas used here:
// - Eq. (1): v' = v - (v · n) n
// - Eq. (2): tangential friction magnitude reduction
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
        // Match python_sim_dev/water_sim_basic.cpp behavior:
        // only hard-project when penetrating the surface (distance < 0).
        // Vertices in the adhesion band [0, adhesionDist] should not be forcibly snapped.
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
            } else {
                vNew -= eps * (vNew / speed);
            }

            // 4) Immediate position correction by velocity change.
            X.row(i) += ((vNew - vOld) * dt).transpose();
            U.row(i) = vNew.transpose();
        } else if (s.signedDistance < adhesionDist) {
            // Adhesion-zone viscous drag: damp the tangential velocity of
            // near-surface vertices.  This stops the curvature-flow/collision
            // cycle from sustaining oscillation at the contact line without
            // constraining contact-angle or normal dynamics.
            // Damping tapers linearly: full at distance=0, zero at adhesionDist.
            const double cld = surface.material().contactLineDamping;
            if (cld > 0.0) {
                const Vec3 n = s.normal.normalized();
                const Vec3 v = U.row(i).transpose();
                const Vec3 vTang = v - v.dot(n) * n;
                const double blend = 1.0 - s.signedDistance / std::max(adhesionDist, 1e-12);
                const double dampFactor = std::max(0.0, 1.0 - cld * dt * blend);
                U.row(i) = (v.dot(n) * n + vTang * dampFactor).transpose();
            }
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
// Zhang et al. force decomposition term used here:
// f_st,i = gamma * Delta x_i
// a_st,i = f_st,i / rho
// v_i <- v_i + a_st,i * dt
// (same discrete form as the working python_sim_dev/water_sim_basic.cpp path)
void CurvatureFlowOperator::apply(Droplet& drop, const ISurface&, double dt) const {
    const auto& X = drop.positions();
    auto& U = drop.velocities();
    const auto& N = drop.derived().vertexNormals;
    if (X.rows() == 0 || dt <= 0.0) return;

    const auto neighbours = buildNeighbours(drop.faces(), X.rows());
    const Weights w = computeCotangentWeights(X, drop.faces());
    const MatX3d lapX = computeLaplacian(X, neighbours, w);

    // Zhang et al. volume-preservation pressure term:
    // f_vol,i = k_v (V0 - V) n_i,  a_vol,i = f_vol,i / rho.
    // We apply it in the same acceleration accumulation path as curvature force.
    const double V0 = (drop.targetVolume() > 0.0) ? drop.targetVolume() : drop.derived().restVolume;
    const double V = drop.derived().currentVolume;
    const double volumeStiffness = std::max(0.0, drop.material().volumeStiffness); // k_v
    const double vp = volumeStiffness * (V0 - V) / std::max(drop.material().density, 1e-8);

    const double scale = (drop.material().surfaceTension / std::max(drop.material().density, 1e-8)) * dt;
    U += scale * lapX;
    for (int i = 0; i < U.rows(); ++i) {
        Vec3 ni = N.row(i).transpose();
        const double nn = ni.norm();
        if (nn <= 1e-12) continue;
        ni /= nn;
        U.row(i) += (vp * dt * ni).transpose();
    }
}

// Contact-angle hysteresis operator (Zhang et al. §4.3).
//
// Contact-line vertices: colliding vertices (within adhesionDist) that neighbor
// at least one non-colliding vertex — the water/air/solid triple junction.
//
// n_L is computed from free-surface faces only (faces with at least one
// non-colliding vertex).  Using all adjacent faces biases n_L toward (0,-1,0)
// due to the flat contact-patch triangles at y=0, making computed θ appear
// smaller than the true geometric angle and causing the operator to under-fire.
//
// θ < θ_r  → retract (−n_p_dir)   θ > θ_a  → advance (+n_p_dir)
void ContactLineOperator::apply(Droplet& drop, const ISurface& surface, double dt, double adhesionDist) const {
    auto& U = drop.velocities();
    const auto& X = drop.positions();
    const auto& F = drop.faces();
    const int nVerts = static_cast<int>(X.rows());

    const SurfaceMaterialParams& surfaceMaterial = surface.material();
    const double alpha   = surfaceMaterial.contactStiffness;
    const double theta_r = surfaceMaterial.recContactAngleDeg * kPi / 180.0;
    const double theta_a = surfaceMaterial.advContactAngleDeg * kPi / 180.0;
    const double density = std::max(drop.material().density, 1e-8);

    if (theta_r > theta_a) return;

    // Pass 1: classify colliding vertices.
    std::vector<bool> colliding(nVerts, false);
    for (int i = 0; i < nVerts; ++i) {
        const SurfaceSample s = surface.closestSample(X.row(i).transpose());
        if (s.signedDistance < adhesionDist) colliding[i] = true;
    }

    // Vertex-to-face map for per-vertex local normal computation.
    std::vector<std::vector<int>> vertFaces(nVerts);
    for (int fi = 0; fi < F.rows(); ++fi) {
        vertFaces[F(fi, 0)].push_back(fi);
        vertFaces[F(fi, 1)].push_back(fi);
        vertFaces[F(fi, 2)].push_back(fi);
    }

    const auto neighbours = buildNeighbours(F, nVerts);

    for (int i = 0; i < nVerts; ++i) {
        if (!colliding[i]) continue;
        bool isContactLine = false;
        for (int nb : neighbours[i]) {
            if (!colliding[nb]) { isContactLine = true; break; }
        }
        if (!isContactLine) continue;

        // n_L from free-surface faces only (paper §4.3: faces A,B,C between
        // water and air, not the flat contact-patch faces).
        Vec3 n_L = Vec3::Zero();
        for (int fi : vertFaces[i]) {
            const int a = F(fi, 0), b = F(fi, 1), c = F(fi, 2);
            if (colliding[a] && colliding[b] && colliding[c]) continue;
            n_L += (X.row(b) - X.row(a)).transpose().cross(
                    (X.row(c) - X.row(a)).transpose());
        }
        const double n_L_norm = n_L.norm();
        if (n_L_norm < kEps) continue;
        n_L /= n_L_norm;

        const SurfaceSample s = surface.closestSample(X.row(i).transpose());
        const Vec3 n_i     = s.normal.normalized();
        const double dotVal = std::clamp(n_L.dot(n_i), -1.0, 1.0);
        const double theta  = kPi - std::acos(dotVal);

        const Vec3   n_p      = n_L - dotVal * n_i;
        const double n_p_norm = n_p.norm();
        if (n_p_norm < kEps) continue;
        const Vec3 n_p_dir = n_p / n_p_norm;

        if (theta >= theta_r && theta <= theta_a) continue;

        Vec3 fBoundary;
        if (theta < theta_r) {
            fBoundary = alpha * (theta - theta_r) * n_p_dir;
        } else {
            fBoundary = alpha * (theta - theta_a) * n_p_dir;
        }
        U.row(i) += ((fBoundary / density) * dt).transpose();
    }
}

// Public wrapper for closed-volume estimate.
double VolumeCorrector::computeClosedVolume(const Droplet& drop, const ISurface&) const {
    return computeClosedVolumeLocal(drop);
}

// Two-stage volume stabilization:
// 1) Local velocity correction (reduce volume-changing deformation modes).
// 2) Global positional correction to hit target volume exactly.
// Zhang et al. formulas used here:
// - Eq. (10)-(11): area-weighted local normal-velocity averaging/correction
// - Global correction: d = Delta V / A, x_i <- x_i + d n_i
void VolumeCorrector::apply(Droplet& drop, const ISurface& surface, double dt, double adhesionDist) const {
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

    // Global volume correction: d = ΔV / A_free, then x_i += d n_i for free vertices only.
    // Contact-zone vertices are excluded: their mesh normals often point partially INTO the
    // surface, so a positive correction (d > 0) would push them through the plane and trigger
    // a collision snap next substep — a pumping cycle that drives the burst oscillation.
    if (doGlobal) {
        // Classify each vertex: free (outside adhesion zone) or contact (inside).
        std::vector<bool> isFree(n, true);
        double freeArea = 0.0;
        if (adhesionDist > 0.0) {
            for (int i = 0; i < n; ++i) {
                const SurfaceSample s = surface.closestSample(X.row(i).transpose());
                if (s.signedDistance < adhesionDist) {
                    isFree[i] = false;
                } else {
                    freeArea += lumpedAreas[i];
                }
            }
        } else {
            freeArea = sumArea;
        }
        // Fallback: if every vertex is in the adhesion zone, correct all (old behavior).
        if (freeArea < 1e-12) {
            freeArea = sumArea;
            std::fill(isFree.begin(), isFree.end(), true);
        }

        double d = dV / freeArea;
        const double edgeScale = std::max(drop.derived().meanEdgeLength, 1e-8);
        const double maxStep = 0.25 * edgeScale;
        d = std::clamp(d, -maxStep, maxStep);
        for (int i = 0; i < n; ++i) {
            if (!isFree[i]) continue;
            Vec3 ni = N.row(i).transpose();
            const double nn = ni.norm();
            if (nn <= 1e-12) continue;
            ni /= nn;
            X.row(i) += (d * ni).transpose();
        }
    }
}

} // namespace wd
