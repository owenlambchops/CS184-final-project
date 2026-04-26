import numpy as np
from scipy.spatial import ConvexHull
from scipy.sparse import csr_matrix
from collections import defaultdict


def normalise(vec):
    """Normalize a vector or array of vectors."""
    norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    out = np.array(vec, copy=True)
    np.divide(vec, norm, out=out, where=norm > 0)
    return out


def generate_points_on_sphere(n_pts=100, r=1.0):
    """Generates uniformly distributed points on a sphere (Fibonacci method)."""
    """https://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/"""
    goldenRatio = (1 + np.sqrt(5)) / 2
    idx = np.linspace(0, n_pts - 1, n_pts)
    theta = 2 * np.pi * idx / goldenRatio
    phi = np.arccos(1 - 2 * idx / n_pts)
    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)
    return (np.stack((x, y, z), axis=1) * r).astype(np.float32)


def generate_polyhedron(n_vertices=40, radius=1.0):
    """Generates a convex polyhedron approximating a sphere."""
    points = generate_points_on_sphere(n_pts=n_vertices, r=radius)
    hull = ConvexHull(points)
    x = points.astype(np.float32)
    faces = hull.simplices

    neighbours = {i: set() for i in range(len(x))}
    for tri in faces:
        for k in range(3):
            i = tri[k]
            j = tri[(k + 1) % 3]
            neighbours[i].add(j)
            neighbours[j].add(i)

    return x, faces, {i: sorted(list(nbs)) for i, nbs in neighbours.items()}


def compute_cotangent_weights(x, faces):
    """Computes cotangent weights for Laplace-Beltrami discretization."""
    w = defaultdict(dict)
    x64 = np.asarray(x, dtype=np.float64)
    x0, x1, x2 = x64[faces[:, 0]], x64[faces[:, 1]], x64[faces[:, 2]]

    cross0 = np.cross(x1 - x0, x2 - x0)
    cross1 = np.cross(x2 - x1, x0 - x1)
    cross2 = np.cross(x0 - x2, x1 - x2)

    norm0 = np.linalg.norm(cross0, axis=1)
    norm1 = np.linalg.norm(cross1, axis=1)
    norm2 = np.linalg.norm(cross2, axis=1)

    norm0 = np.where(norm0 == 0, 1e-8, norm0)
    norm1 = np.where(norm1 == 0, 1e-8, norm1)
    norm2 = np.where(norm2 == 0, 1e-8, norm2)

    cot0 = np.einsum("ij,ij->i", x1 - x0, x2 - x0) / norm0
    cot1 = np.einsum("ij,ij->i", x2 - x1, x0 - x1) / norm1
    cot2 = np.einsum("ij,ij->i", x0 - x2, x1 - x2) / norm2

    # Cap cotangents to avoid explosive forces on near-degenerate triangles.
    cot_cap = 50.0
    cot0 = np.clip(cot0, 0.0, cot_cap)
    cot1 = np.clip(cot1, 0.0, cot_cap)
    cot2 = np.clip(cot2, 0.0, cot_cap)

    for f, (i0, i1, i2) in enumerate(faces):
        w0 = max(0.0, cot2[f] / 2.0)
        w1 = max(0.0, cot0[f] / 2.0)
        w2 = max(0.0, cot1[f] / 2.0)

        w[i0][i1] = w[i0].get(i1, 0.0) + w0
        w[i1][i0] = w[i1].get(i0, 0.0) + w0
        w[i1][i2] = w[i1].get(i2, 0.0) + w1
        w[i2][i1] = w[i2].get(i1, 0.0) + w1
        w[i2][i0] = w[i2].get(i0, 0.0) + w2
        w[i0][i2] = w[i0].get(i2, 0.0) + w2

    return w


def cotangent_weights_to_sparse(w, n_vertices, dtype=np.float64):
    """Convert cotangent weight dict to sparse Laplacian matrix."""
    row, col, data = [], [], []
    for i in range(n_vertices):
        row_sum = 0.0
        for j, wij in w[i].items():
            row.append(i)
            col.append(j)
            data.append(wij)
            row_sum += wij
        row.append(i)
        col.append(i)
        data.append(-row_sum)
    return csr_matrix((np.asarray(data, dtype=dtype), (row, col)), shape=(n_vertices, n_vertices))


def compute_lumped_areas(x, faces):
    """Compute lumped area for each vertex (Voronoi area approximation)."""
    n_vertices = len(x)
    areas = np.zeros(n_vertices, dtype=np.float32)
    for face in faces:
        v0, v1, v2 = x[face[0]], x[face[1]], x[face[2]]
        face_area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
        areas[face] += face_area / 3.0
    return areas


def compute_lumped_masses(x, faces, rho=1.0):
    """Lumped mass matrix (area * density)."""
    return rho * compute_lumped_areas(x, faces)


def compute_vertex_normals_and_volume(x, faces):
    """Calculate vertex normals and internal volume."""
    x64 = np.asarray(x, dtype=np.float64)
    n = np.zeros_like(x64)
    V = 0.0
    com = np.mean(x64, axis=0)

    for i0, i1, i2 in faces:
        x0, x1, x2 = x64[i0], x64[i1], x64[i2]
        n_face = np.cross(x1 - x0, x2 - x0)

        if np.dot(n_face, x0 - com) < 0:
            n_face, x1, x2 = -n_face, x2, x1

        n[i0] += n_face
        n[i1] += n_face
        n[i2] += n_face
        V += np.dot(x0 - com, np.cross(x1 - com, x2 - com)) / 6.0

    norms = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(norms == 0, 1.0, norms), abs(V)


def build_laplacian_matrix(x, faces, cotangent_weights=None):
    """Build discrete Laplace-Beltrami operator as sparse matrix."""
    if cotangent_weights is None:
        cotangent_weights = compute_cotangent_weights(x, faces)
    return cotangent_weights_to_sparse(cotangent_weights, len(x), dtype=np.float64)


def compute_laplacians(x, v, neighbours=None, w=None, laplacian_matrix=None):
    """
    Compute graph Laplacian of positions (Δx) and velocities (Δv)
    using sparse matrix multiplication.
    """
    if laplacian_matrix is None:
        if w is None:
            raise ValueError("Provide either laplacian_matrix or cotangent weights w.")
        laplacian_matrix = cotangent_weights_to_sparse(w, len(x), dtype=np.float64)

    x64 = np.asarray(x, dtype=np.float64)
    v64 = np.asarray(v, dtype=np.float64)
    delta_x = laplacian_matrix @ x64
    delta_v = laplacian_matrix @ v64
    delta_x = np.asarray(delta_x, dtype=x.dtype)
    delta_v = np.asarray(delta_v, dtype=v.dtype)
    return delta_x, delta_v


def boundary_force(
    alpha,
    n_l,
    contact_mask,
    receding_angle,
    advancing_angle,
    ground_normal=None,
):
    """
    Compute contact‑angle hysteresis boundary force (implements a penalty
    that tries to keep the angle between receding and advancing angles).
    """
    if receding_angle > advancing_angle:
        raise ValueError("receding_angle must be <= advancing_angle")

    f_boundary = np.zeros_like(n_l)
    if np.count_nonzero(contact_mask) == 0:
        return f_boundary

    if ground_normal is None:
        ground_normal = np.array([0.0, 0.0, 1.0], dtype=n_l.dtype)
    n_i = normalise(np.asarray(ground_normal, dtype=n_l.dtype))
    contact_ids = np.where(contact_mask)[0]

    for i in contact_ids:
        n_li = n_l[i]
        dot_val = np.clip(np.dot(n_li, n_i), -1.0, 1.0)
        angle = np.arccos(dot_val)

        # Projected direction in the tangent plane
        n_p = n_li - dot_val * n_i
        n_p_dir = normalise(n_p)
        if np.linalg.norm(n_p_dir) < 1e-8:
            continue

        if angle < receding_angle:
            f_boundary[i] = alpha * (angle - receding_angle) * n_p_dir
        elif angle > advancing_angle:
            f_boundary[i] = alpha * (angle - advancing_angle) * n_p_dir
        # else: no force (hysteresis region)

    return f_boundary


def compute_total_acceleration(
    x,
    v,
    faces,
    neighbours,
    V_0,
    g,
    gamma,
    k_v,
    boundary_alpha,
    receding_angle,
    advancing_angle,
    adhesion_dist,
    max_internal_accel,
    density,
):
    """
    Compute total acceleration a_total and velocity Laplacian Δv.
    """
    if density <= 0:
        raise ValueError("density must be > 0")

    w = compute_cotangent_weights(x, faces)
    L = build_laplacian_matrix(x, faces, cotangent_weights=w)
    n, V = compute_vertex_normals_and_volume(x, faces)
    delta_x, delta_v = compute_laplacians(x, v, laplacian_matrix=L)

    # Surface tension (mean curvature flow) as acceleration
    f_st = gamma * delta_x

    # Volume preservation force (soft constraint)
    f_vol = k_v * (V_0 - V) * n

    # Contact‑angle boundary force
    contact_mask = x[:, 2] <= adhesion_dist
    f_boundary = boundary_force(
        boundary_alpha,
        n,
        contact_mask,
        receding_angle,
        advancing_angle,
    )

    a_ext = np.broadcast_to(g, x.shape)
    a_internal = (f_st + f_vol + f_boundary) / density

    # Limit excessive internal accelerations for stability
    a_int_norm = np.linalg.norm(a_internal, axis=1, keepdims=True)
    scale = np.minimum(1.0, max_internal_accel / np.maximum(a_int_norm, 1e-8))
    a_internal = a_internal * scale

    a_total = a_ext + a_internal
    a_total = np.nan_to_num(a_total, nan=0.0, posinf=0.0, neginf=0.0)
    delta_v = np.nan_to_num(delta_v, nan=0.0, posinf=0.0, neginf=0.0)
    return a_total, delta_v


def apply_surface_interactions(x, v, dt, friction_coeff, adhesion_dist):
    """
    Handle ground (z=0) collisions and Coulomb friction.
    Also applies a small damping to horizontal motion.
    """
    x = x.copy()
    v = v.copy()

    contact_mask = x[:, 2] <= adhesion_dist
    if not np.any(contact_mask):
        return x, v

    below_ground = x[:, 2] < 0
    x[below_ground, 2] = 0.0

    downward_impact = below_ground & (v[:, 2] < 0)
    v[downward_impact, 2] *= -0.2  # inelastic bounce

    horiz = v[contact_mask, :2]
    speed = np.linalg.norm(horiz, axis=1, keepdims=True)
    drop = friction_coeff * dt
    scale = np.zeros_like(speed)
    np.divide(np.maximum(speed - drop, 0.0), speed, out=scale, where=speed > 1e-12)
    v[contact_mask, :2] = horiz * scale

    # Additional mild damping to prevent numerical drift
    v[contact_mask, :2] *= max(0.0, 1.0 - 0.05 * dt)

    return x, v


def apply_local_volume_correction(x, v, faces, density=1.0, rho=None):
    """
    Remove the divergent component of velocity field using area‑weighted
    averaging of the normal component (ensures local volume preservation).

    `rho` is kept as a backward-compatible alias for `density`.
    """
    if rho is not None:
        density = rho

    n, _ = compute_vertex_normals_and_volume(x, faces)
    masses = compute_lumped_masses(x, faces, density)

    # Normal component of velocity
    a_i = np.sum(v * n, axis=1)

    # Build undirected adjacency once, then smooth via sparse matmul.
    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])).astype(np.int32)
    row = np.concatenate((edges[:, 0], edges[:, 1]))
    col = np.concatenate((edges[:, 1], edges[:, 0]))
    data = np.ones(row.shape[0], dtype=np.float64)
    A = csr_matrix((data, (row, col)), shape=(len(x), len(x)))
    A.data[:] = 1.0  # collapse duplicated edges to uniform 1.0 weight
    A.eliminate_zeros()

    weighted_vals = masses * a_i
    denom = np.asarray(A @ masses, dtype=np.float64)
    numer = np.asarray(A @ weighted_vals, dtype=np.float64)
    a_bar = np.where(denom > 1e-8, numer / denom, 0.0)

    # Correct velocity
    v_corrected = v - a_bar[:, None] * n
    return v_corrected


def apply_global_volume_correction(
    x, faces, V_target, volume_stiffness=1.0, ground_mask=None
):
    """
    Global volume correction: move vertices along normals to restore V_target.
    If ground_mask is provided, those vertices are only moved horizontally
    (their vertical component is zeroed) to avoid lifting the base.
    """
    n, V = compute_vertex_normals_and_volume(x, faces)
    delta_V = V_target - V
    if abs(delta_V) < 1e-10:
        return x

    total_area = 0.0
    for face in faces:
        x0, x1, x2 = x[face[0]], x[face[1]], x[face[2]]
        total_area += 0.5 * np.linalg.norm(np.cross(x1 - x0, x2 - x0))

    if total_area < 1e-8:
        return x

    # Maximum step to avoid overshoot (relative to average edge length)
    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edge_len = np.linalg.norm(x[edges[:, 0]] - x[edges[:, 1]], axis=1)
    d_max = 0.25 * np.mean(edge_len) if edge_len.size > 0 else 1e-3

    stiffness = np.clip(volume_stiffness, 0.0, 1.0)
    d = stiffness * (delta_V / total_area)
    d = np.clip(d, -d_max, d_max)

    disp = d * n

    # If ground_mask is provided, restrict ground vertices to horizontal movement only
    if ground_mask is not None:
        # Zero out vertical component for ground vertices
        disp[ground_mask, 2] = 0.0

    x_corrected = x + disp
    return x_corrected


def step_forward_euler(
    v,
    x,
    faces,
    neighbours,
    V_0,
    dt,
    g,
    gamma,
    mu,
    eta,
    k_v,
    friction_coeff,
    boundary_alpha=0.5,
    receding_angle=np.deg2rad(70.0),
    advancing_angle=np.deg2rad(95.0),
    adhesion_dist=0.05,
    max_internal_accel=200.0,
    density=1.0,
    volume_stiffness=0.5,
):
    """
    Explicit Euler time step with local+global volume correction.
    """
    x = np.asarray(x, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)

    # 1. Compute accelerations (includes surface tension, volume force, contact angle)
    a_total, delta_v = compute_total_acceleration(
        x,
        v,
        faces,
        neighbours,
        V_0,
        g,
        gamma,
        k_v,
        boundary_alpha=boundary_alpha,
        receding_angle=receding_angle,
        advancing_angle=advancing_angle,
        adhesion_dist=adhesion_dist,
        max_internal_accel=max_internal_accel,
        density=density,
    )

    # 2. Damping and explicit Euler update
    v_damped = (v + (eta * dt * delta_v)) / (1.0 + mu * dt)
    v_new = np.clip(v_damped + a_total * dt, -15.0, 15.0)
    x_new = x + v_new * dt

    # 3. Ground collision and friction
    x_new, v_new = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, adhesion_dist
    )

    # 4. Local volume correction (remove compressible part of velocity)
    v_new = apply_local_volume_correction(x_new, v_new, faces, density=density)

    # 5. Global volume correction (move vertices to restore exact volume)
    ground_mask = x_new[:, 2] <= adhesion_dist  # vertices in contact with ground
    x_new = apply_global_volume_correction(
        x_new,
        faces,
        V_0,
        volume_stiffness=volume_stiffness,
        ground_mask=ground_mask,
    )

    # 6. Re‑apply ground constraints after volume correction (avoid penetrating)
    x_new, v_new = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, adhesion_dist
    )

    return v_new, x_new


# Optional: Verlet integrator (more stable, but slower)
def step_verlet(
    v,
    x,
    faces,
    neighbours,
    V_0,
    dt,
    g,
    gamma,
    mu,
    eta,
    k_v,
    friction_coeff,
    boundary_alpha=0.5,
    receding_angle=np.deg2rad(70.0),
    advancing_angle=np.deg2rad(95.0),
    adhesion_dist=0.05,
    max_internal_accel=200.0,
    max_internal_substeps=8,
    density=1.0,
    volume_stiffness=0.5,
):
    """
    Velocity‑Verlet integrator with adaptive substeps and volume correction.
    """
    x = np.asarray(x, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)

    # Adaptive substeps based on expected maximum acceleration
    edge_len = compute_reference_edge_length(x, faces)
    a_probe, _ = compute_total_acceleration(
        x,
        v,
        faces,
        neighbours,
        V_0,
        g,
        gamma,
        k_v,
        boundary_alpha=boundary_alpha,
        receding_angle=receding_angle,
        advancing_angle=advancing_angle,
        adhesion_dist=adhesion_dist,
        max_internal_accel=max_internal_accel,
        density=density,
    )
    a_max = float(np.max(np.linalg.norm(a_probe, axis=1)))
    if a_max > 1e-8:
        dt_safe = 0.35 * np.sqrt(edge_len / a_max)
        internal_substeps = int(
            np.clip(np.ceil(dt / max(dt_safe, 1e-6)), 1, max_internal_substeps)
        )
    else:
        internal_substeps = 1

    dt_sub = dt / internal_substeps
    x_curr = x.copy()
    v_curr = v.copy()

    for _ in range(internal_substeps):
        a_n, delta_v_n = compute_total_acceleration(
            x_curr,
            v_curr,
            faces,
            neighbours,
            V_0,
            g,
            gamma,
            k_v,
            boundary_alpha=boundary_alpha,
            receding_angle=receding_angle,
            advancing_angle=advancing_angle,
            adhesion_dist=adhesion_dist,
            max_internal_accel=max_internal_accel,
            density=density,
        )

        damp_half = 1.0 + 0.5 * mu * dt_sub
        v_half = (
            v_curr + 0.5 * dt_sub * a_n + 0.5 * eta * dt_sub * delta_v_n
        ) / damp_half
        v_half = np.nan_to_num(v_half, nan=0.0, posinf=0.0, neginf=0.0)

        x_next = x_curr + dt_sub * v_half
        x_next = np.nan_to_num(x_next, nan=0.0, posinf=0.0, neginf=0.0)

        a_np1, delta_v_np1 = compute_total_acceleration(
            x_next,
            v_half,
            faces,
            neighbours,
            V_0,
            g,
            gamma,
            k_v,
            boundary_alpha=boundary_alpha,
            receding_angle=receding_angle,
            advancing_angle=advancing_angle,
            adhesion_dist=adhesion_dist,
            max_internal_accel=max_internal_accel,
            density=density,
        )

        v_next = (
            v_half + 0.5 * dt_sub * a_np1 + 0.5 * eta * dt_sub * delta_v_np1
        ) / damp_half
        v_next = np.clip(
            np.nan_to_num(v_next, nan=0.0, posinf=0.0, neginf=0.0), -15.0, 15.0
        )

        # Collision and friction
        x_next, v_next = apply_surface_interactions(
            x_next, v_next, dt_sub, friction_coeff, adhesion_dist
        )

        # Volume corrections
        v_next = apply_local_volume_correction(x_next, v_next, faces, density=density)
        ground_mask = x_next[:, 2] <= adhesion_dist
        x_next = apply_global_volume_correction(
            x_next,
            faces,
            V_0,
            volume_stiffness=volume_stiffness,
            ground_mask=ground_mask,
        )
        x_next, v_next = apply_surface_interactions(
            x_next, v_next, dt_sub, friction_coeff, adhesion_dist
        )

        x_curr = x_next
        v_curr = v_next

    return v_curr, x_curr


def compute_reference_edge_length(x, faces):
    """Median edge length as a reference for time stepping."""
    if len(faces) == 0:
        return 1e-3
    e01 = np.linalg.norm(x[faces[:, 0]] - x[faces[:, 1]], axis=1)
    e12 = np.linalg.norm(x[faces[:, 1]] - x[faces[:, 2]], axis=1)
    e20 = np.linalg.norm(x[faces[:, 2]] - x[faces[:, 0]], axis=1)
    edges = np.concatenate((e01, e12, e20))
    edges = edges[edges > 1e-8]
    if edges.size == 0:
        return 1e-3
    return float(np.median(edges))


def step_simulation(
    v,
    x,
    faces,
    neighbours,
    V_0,
    dt,
    method="euler",
    g=None,
    gamma=0.2,
    mu=0.3,
    eta=0.05,
    k_v=200.0,
    density=1.0,
    boundary_alpha=0.5,
    friction_coeff=0.5,
    receding_angle=np.deg2rad(70.0),
    advancing_angle=np.deg2rad(95.0),
    adhesion_dist=0.05,
    max_internal_accel=200.0,
    max_internal_substeps=8,
    volume_stiffness=0.5,
    rho=None,
):
    """Unified step function with safer defaults and method switch."""
    if g is None:
        g = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    if rho is not None:
        density = rho

    if method.lower() == "euler":
        return step_forward_euler(
            v,
            x,
            faces,
            neighbours,
            V_0,
            dt,
            g,
            gamma,
            mu,
            eta,
            k_v,
            friction_coeff,
            boundary_alpha=boundary_alpha,
            receding_angle=receding_angle,
            advancing_angle=advancing_angle,
            adhesion_dist=adhesion_dist,
            max_internal_accel=max_internal_accel,
            density=density,
            volume_stiffness=volume_stiffness,
        )
    if method.lower() == "verlet":
        return step_verlet(
            v,
            x,
            faces,
            neighbours,
            V_0,
            dt,
            g,
            gamma,
            mu,
            eta,
            k_v,
            friction_coeff,
            boundary_alpha=boundary_alpha,
            receding_angle=receding_angle,
            advancing_angle=advancing_angle,
            adhesion_dist=adhesion_dist,
            max_internal_accel=max_internal_accel,
            max_internal_substeps=max_internal_substeps,
            density=density,
            volume_stiffness=volume_stiffness,
        )
    raise ValueError(f"Unknown method: {method}")
