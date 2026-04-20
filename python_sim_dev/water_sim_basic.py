import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation
from collections import defaultdict


def normalise(vec):
    norm = np.linalg.norm(vec)
    return vec / norm if norm > 0 else vec


def generate_points_on_sphere(n_pts=100, r=1.0):
    """generates a uniformly distributed set of points on the surface of a sphere using the Fibonacci method.
    https://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/
    """
    goldenRatio = (1 + np.sqrt(5)) / 2
    idx = np.linspace(0, n_pts - 1, n_pts)
    theta = 2 * np.pi * idx / goldenRatio
    phi = np.arccos(1 - 2 * (idx) / n_pts)
    x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)
    return (np.stack((x, y, z), axis=1) * r).astype(np.float32)


def generate_polyhedron(n_vertices=40, radius=1.0):
    """Generates a random convex polyhedron representing the initial droplet."""
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


def generate_rectangular_cube_mesh(
    width=1.0,
    height=1.0,
    length=1.0,
    nx=5,
    ny=5,
    nz=5,
    center=(0.0, 0.0, 0.0),
):
    """Generates a triangulated rectangular cuboid surface mesh.

    Axis convention:
        - width along X
        - length along Y
        - height along Z

    Args:
        width (float): Cuboid size along X.
        height (float): Cuboid size along Z.
        length (float): Cuboid size along Y.
        nx (int): Boundary resolution along X (must be >= 2).
        ny (int): Boundary resolution along Y (must be >= 2).
        nz (int): Boundary resolution along Z (must be >= 2).
        center (tuple[float, float, float]): Cuboid center in 3D.

    Returns:
        tuple: (x, faces, neighbours)
            x: (N, 3) float32 vertex positions.
            faces: (M, 3) int32 triangle indices.
            neighbours: dict[int, list[int]] adjacency list.
    """
    if nx < 2 or ny < 2 or nz < 2:
        raise ValueError("nx, ny, nz must all be at least 2")

    cx, cy, cz = center
    xs = np.linspace(cx - width * 0.5, cx + width * 0.5, nx, dtype=np.float32)
    ys = np.linspace(cy - length * 0.5, cy + length * 0.5, ny, dtype=np.float32)
    zs = np.linspace(cz - height * 0.5, cz + height * 0.5, nz, dtype=np.float32)

    vertex_map = {}
    vertices = []

    def is_boundary(i, j, k):
        return i == 0 or i == nx - 1 or j == 0 or j == ny - 1 or k == 0 or k == nz - 1

    def vid(i, j, k):
        key = (i, j, k)
        if key in vertex_map:
            return vertex_map[key]
        if not is_boundary(i, j, k):
            raise ValueError("Requested an interior vertex for a surface mesh")
        vertex_map[key] = len(vertices)
        vertices.append((xs[i], ys[j], zs[k]))
        return vertex_map[key]

    faces = []

    def add_quad(v00, v10, v11, v01, flip=False):
        if flip:
            faces.append((v00, v11, v10))
            faces.append((v00, v01, v11))
        else:
            faces.append((v00, v10, v11))
            faces.append((v00, v11, v01))

    # Z-min and Z-max faces
    for j in range(ny - 1):
        for i in range(nx - 1):
            add_quad(
                vid(i, j, 0),
                vid(i + 1, j, 0),
                vid(i + 1, j + 1, 0),
                vid(i, j + 1, 0),
                flip=True,
            )
            add_quad(
                vid(i, j, nz - 1),
                vid(i + 1, j, nz - 1),
                vid(i + 1, j + 1, nz - 1),
                vid(i, j + 1, nz - 1),
                flip=False,
            )

    # Y-min and Y-max faces
    for k in range(nz - 1):
        for i in range(nx - 1):
            add_quad(
                vid(i, 0, k),
                vid(i + 1, 0, k),
                vid(i + 1, 0, k + 1),
                vid(i, 0, k + 1),
                flip=False,
            )
            add_quad(
                vid(i, ny - 1, k),
                vid(i + 1, ny - 1, k),
                vid(i + 1, ny - 1, k + 1),
                vid(i, ny - 1, k + 1),
                flip=True,
            )

    # X-min and X-max faces
    for k in range(nz - 1):
        for j in range(ny - 1):
            add_quad(
                vid(0, j, k),
                vid(0, j + 1, k),
                vid(0, j + 1, k + 1),
                vid(0, j, k + 1),
                flip=True,
            )
            add_quad(
                vid(nx - 1, j, k),
                vid(nx - 1, j + 1, k),
                vid(nx - 1, j + 1, k + 1),
                vid(nx - 1, j, k + 1),
                flip=False,
            )

    x = np.asarray(vertices, dtype=np.float32)
    faces = np.asarray(faces, dtype=np.int32)

    neighbours = {i: set() for i in range(len(x))}
    for tri in faces:
        for k in range(3):
            i = int(tri[k])
            j = int(tri[(k + 1) % 3])
            neighbours[i].add(j)
            neighbours[j].add(i)

    return x, faces, {i: sorted(list(nbs)) for i, nbs in neighbours.items()}


def generate_rectangle_mesh(
    width=1.0,
    height=1.0,
    nx=20,
    ny=20,
    center=(0.0, 0.0, 0.0),
):
    """Backward-compatible wrapper that now generates a thin cuboid mesh."""
    thickness = min(width, height) * 0.05
    return generate_rectangular_cube_mesh(
        width=width,
        height=thickness,
        length=height,
        nx=nx,
        ny=ny,
        nz=2,
        center=center,
    )


def compute_cotangent_weights(x, faces):
    """Computes weights w_ij to approximate the Laplace-Beltrami operator"""
    w = defaultdict(dict)
    x0, x1, x2 = x[faces[:, 0]], x[faces[:, 1]], x[faces[:, 2]]

    cross0, cross1, cross2 = (
        np.cross(x1 - x0, x2 - x0),
        np.cross(x2 - x1, x0 - x1),
        np.cross(x0 - x2, x1 - x2),
    )

    norm0 = np.where(
        np.linalg.norm(cross0, axis=1) == 0, 1e-5, np.linalg.norm(cross0, axis=1)
    )
    norm1 = np.where(
        np.linalg.norm(cross1, axis=1) == 0, 1e-5, np.linalg.norm(cross1, axis=1)
    )
    norm2 = np.where(
        np.linalg.norm(cross2, axis=1) == 0, 1e-5, np.linalg.norm(cross2, axis=1)
    )

    cot0 = np.einsum("ij,ij->i", x1 - x0, x2 - x0) / norm0
    cot1 = np.einsum("ij,ij->i", x2 - x1, x0 - x1) / norm1
    cot2 = np.einsum("ij,ij->i", x0 - x2, x1 - x2) / norm2

    for f, (i0, i1, i2) in enumerate(faces):
        w0, w1, w2 = (
            max(0.0, cot2[f] / 2.0),
            max(0.0, cot0[f] / 2.0),
            max(0.0, cot1[f] / 2.0),
        )

        w[i0][i1] = w[i0].get(i1, 0.0) + w0
        w[i1][i0] = w[i1].get(i0, 0.0) + w0
        w[i1][i2] = w[i1].get(i2, 0.0) + w1
        w[i2][i1] = w[i2].get(i1, 0.0) + w1
        w[i2][i0] = w[i2].get(i0, 0.0) + w2
        w[i0][i2] = w[i0].get(i2, 0.0) + w2

    return w


def compute_laplacians(x, v, neighbours, w):
    """Computes positional (Delta x) and velocity (Delta v) laplacians for all vertices."""
    delta_x = np.zeros_like(x)
    delta_v = np.zeros_like(v)

    for i in range(len(x)):
        sum_w = 0.0
        for j in neighbours[i]:
            w_ij = w[i].get(j, 1.0)
            delta_x[i] += w_ij * (x[j] - x[i])
            delta_v[i] += w_ij * (v[j] - v[i])
            sum_w += w_ij

        if sum_w > 0:
            delta_x[i] /= sum_w
            delta_v[i] /= sum_w

    return delta_x, delta_v


def compute_vertex_normals_and_volume(x, faces):
    """Calculates vertex normals (n_i) and the exact internal volume (V) of the mesh."""
    n = np.zeros_like(x)
    V = 0.0
    com = np.mean(x, axis=0)

    for i0, i1, i2 in faces:
        x0, x1, x2 = x[i0], x[i1], x[i2]
        n_face = np.cross(x1 - x0, x2 - x0)

        if np.dot(n_face, x0 - com) < 0:
            n_face, x1, x2 = -n_face, x2, x1

        n[i0] += n_face
        n[i1] += n_face
        n[i2] += n_face
        V += np.dot(x0 - com, np.cross(x1 - com, x2 - com)) / 6.0

    norms = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.where(norms == 0, 1.0, norms), abs(V)


def apply_surface_interactions(x, v, dt, friction_coeff, adhesion_dist):
    """Handles Z=0 plane collisions, boundary constraints, and sliding friction."""
    for i in range(len(x)):
        if x[i][2] <= adhesion_dist:
            # 1. Collision & Bounce
            if x[i][2] < 0:
                x[i][2] = 0
                if v[i][2] < 0:
                    v[i][2] *= -0.2

            # 2. Sliding Friction / Surface Adhesion
            v_horiz = v[i][:2]
            speed = np.linalg.norm(v_horiz)
            if speed > 0:
                drop = friction_coeff * dt
                if speed < drop:
                    v[i][:2] = 0.0
                else:
                    v[i][:2] *= (speed - drop) / speed

            # 3. viscosity damping
            v[i][:2] = (1 - 0.05 * dt) * v[i][
                :2
            ]  # simple linear damping for horizontal velocity
    return x, v


def boundary_force(
    alpha,
    n_l,
    contact_mask,
    receding_angle,
    advancing_angle,
    ground_normal=np.array([0.0, 0.0, 1.0], dtype=np.float32),
):
    """Computes per-vertex contact-angle hysteresis boundary force in radians."""
    if receding_angle > advancing_angle:
        raise ValueError("receding_angle must be <= advancing_angle")

    f_boundary = np.zeros_like(n_l)
    if np.count_nonzero(contact_mask) == 0:
        return f_boundary

    n_i = normalise(ground_normal.astype(n_l.dtype, copy=False))
    contact_ids = np.where(contact_mask)[0]

    for i in contact_ids:
        n_li = n_l[i]
        dot_val = np.clip(np.dot(n_li, n_i), -1.0, 1.0)
        angle = np.arccos(dot_val)

        # Project liquid surface normal onto the ground plane.
        n_p = n_li - dot_val * n_i
        n_p_dir = normalise(n_p)

        if np.linalg.norm(n_p_dir) == 0:
            continue

        if receding_angle < angle < advancing_angle:
            continue  # no boundary force in the hysteresis range
        if angle <= receding_angle:
            f_boundary[i] = alpha * (angle - receding_angle) * n_p_dir
        else:  # angle >= advancing_angle
            f_boundary[i] = alpha * (angle - advancing_angle) * n_p_dir

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
    """Computes total acceleration and velocity Laplacian for a given state."""
    w = compute_cotangent_weights(x, faces)
    n, V = compute_vertex_normals_and_volume(x, faces)
    delta_x, delta_v = compute_laplacians(x, v, neighbours, w)

    f_st = gamma * delta_x
    f_vol = k_v * (V_0 - V) * n
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

    # Cap stiff internal acceleration to avoid blow-ups at large surface tension.
    a_int_norm = np.linalg.norm(a_internal, axis=1, keepdims=True)
    scale = np.minimum(1.0, max_internal_accel / np.maximum(a_int_norm, 1e-8))
    a_internal = a_internal * scale

    a_total = a_ext + a_internal
    a_total = np.nan_to_num(a_total, nan=0.0, posinf=0.0, neginf=0.0)
    delta_v = np.nan_to_num(delta_v, nan=0.0, posinf=0.0, neginf=0.0)
    return a_total, delta_v


def compute_reference_edge_length(x, faces):
    """Returns a robust characteristic edge length for adaptive time stepping."""
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
):
    """Advances the simulation by one explicit Euler time step."""

    # 1-3. Update geometry and compute total acceleration.
    a_total, delta_v = compute_total_acceleration(
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
    )

    # 4. Time Integration
    # Use implicit-like linear damping for better stability at larger mu.
    v_damped = (v + (eta * dt * delta_v)) / (1.0 + mu * dt)

    # Update velocity using the newly scaled acceleration
    v_new = v_damped + a_total * dt
    v_new = np.clip(v_new, -15.0, 15.0)
    x_new = x + v_new * dt

    # 5. Apply Environmental Constraints (Glass surface)
    x_new, v_new = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, adhesion_dist
    )

    return v_new, x_new


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
):
    """Advances the simulation by one velocity-Verlet time step."""

    # Choose an internal micro-step count using an acceleration-based stability rule.
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
        boundary_alpha,
        receding_angle,
        advancing_angle,
        adhesion_dist,
        max_internal_accel,
        density,
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
    x_curr = x
    v_curr = v

    for _ in range(internal_substeps):
        # Acceleration at the beginning of the sub-step.
        a_n, delta_v_n = compute_total_acceleration(
            x_curr,
            v_curr,
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
        )

        damp_half = 1.0 + 0.5 * mu * dt_sub
        v_half = (
            v_curr + 0.5 * dt_sub * a_n + 0.5 * eta * dt_sub * delta_v_n
        ) / damp_half
        v_half = np.nan_to_num(v_half, nan=0.0, posinf=0.0, neginf=0.0)

        x_next = x_curr + dt_sub * v_half
        x_next = np.nan_to_num(x_next, nan=0.0, posinf=0.0, neginf=0.0)

        # Acceleration at the end of the sub-step.
        a_np1, delta_v_np1 = compute_total_acceleration(
            x_next,
            v_half,
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
        )

        v_next = (
            v_half + 0.5 * dt_sub * a_np1 + 0.5 * eta * dt_sub * delta_v_np1
        ) / damp_half
        v_next = np.clip(
            np.nan_to_num(v_next, nan=0.0, posinf=0.0, neginf=0.0), -15.0, 15.0
        )

        x_next, v_next = apply_surface_interactions(
            x_next, v_next, dt_sub, friction_coeff, adhesion_dist
        )

        x_curr = x_next
        v_curr = v_next

    return v_curr, x_curr
