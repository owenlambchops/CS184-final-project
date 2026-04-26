import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.sparse.linalg import cg
from scipy.sparse import csr_matrix, eye as sparse_eye, diags
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation
from collections import defaultdict


def normalise(vec):
    """Normalize a vector or array of vectors."""
    norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    out = np.array(vec, copy=True)
    np.divide(vec, norm, out=out, where=norm > 0)
    return out


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


def compute_cotangent_weights(x, faces):
    """Computes weights w_ij to approximate the Laplace-Beltrami operator"""
    w = defaultdict(dict)
    x0, x1, x2 = x[faces[:, 0]], x[faces[:, 1]], x[faces[:, 2]]

    cross0 = np.cross(x1 - x0, x2 - x0)
    cross1 = np.cross(x2 - x1, x0 - x1)
    cross2 = np.cross(x0 - x2, x1 - x2)

    norm0 = np.linalg.norm(cross0, axis=1)
    norm1 = np.linalg.norm(cross1, axis=1)
    norm2 = np.linalg.norm(cross2, axis=1)

    norm0 = np.where(norm0 == 0, 1e-5, norm0)
    norm1 = np.where(norm1 == 0, 1e-5, norm1)
    norm2 = np.where(norm2 == 0, 1e-5, norm2)

    cot0 = np.einsum("ij,ij->i", x1 - x0, x2 - x0) / norm0
    cot1 = np.einsum("ij,ij->i", x2 - x1, x0 - x1) / norm1
    cot2 = np.einsum("ij,ij->i", x0 - x2, x1 - x2) / norm2

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


def compute_vertex_normals_and_volume(x, faces):
    """Calculate vertex normals and internal volume."""
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


def compute_lumped_masses(x, faces, rho=1.0):
    """Compute lumped mass matrix for triangular mesh."""
    n_vertices = len(x)
    masses = np.zeros(n_vertices, dtype=np.float32)

    def triangle_area(a, b, c):
        return 0.5 * np.linalg.norm(np.cross(b - a, c - a))

    for i in range(n_vertices):
        vertex_mass = 0.0
        for face in faces:
            if i in face:
                a = x[face[0]]
                b = x[face[1]]
                c = x[face[2]]
                vertex_mass += triangle_area(a, b, c) / 3.0
        masses[i] = rho * vertex_mass

    return masses


def build_laplacian_matrix(n_vertices, faces, cotangent_weights):
    """Build discrete Laplace-Beltrami operator as sparse matrix."""
    row, col, data = [], [], []

    for i in range(n_vertices):
        row_sum = 0.0
        for j in cotangent_weights[i]:
            w = cotangent_weights[i][j]
            row.append(i)
            col.append(j)
            data.append(w)
            row_sum += w
        row.append(i)
        col.append(i)
        data.append(-row_sum)

    return csr_matrix((data, (row, col)), shape=(n_vertices, n_vertices))


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


def apply_surface_interactions(x, v, dt, friction_coeff, adhesion_dist):
    """Handles Z=0 plane collisions, boundary constraints, and sliding friction."""
    x = x.copy()
    v = v.copy()

    for i in range(len(x)):
        if x[i][2] <= adhesion_dist:
            if x[i][2] < 0:
                x[i][2] = 0
                if v[i][2] < 0:
                    v[i][2] *= -0.2

            v_horiz = v[i][:2]
            speed = np.linalg.norm(v_horiz)
            if speed > 0:
                drop = friction_coeff * dt
                if speed < drop:
                    v[i][:2] = 0.0
                else:
                    v[i][:2] *= (speed - drop) / speed

            v[i][:2] = (1 - 0.05 * dt) * v[i][:2]

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

        n_p = n_li - dot_val * n_i
        n_p_dir = normalise(n_p)

        if np.linalg.norm(n_p_dir) == 0:
            continue

        if receding_angle < angle < advancing_angle:
            continue
        if angle <= receding_angle:
            f_boundary[i] = alpha * (angle - receding_angle) * n_p_dir
        else:
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

    v_damped = (v + (eta * dt * delta_v)) / (1.0 + mu * dt)
    v_new = np.clip(v_damped + a_total * dt, -15.0, 15.0)
    x_new = x + v_new * dt
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


def apply_external_forces(x, v, dt, g, friction_coeff=0.5):
    """Apply gravity, collision, and friction."""
    x_new = x.copy()
    v_new = v.copy()

    v_new = v_new + g * dt

    x_new = x_new + v_new * dt

    contact_band = 0.05
    colliding = x_new[:, 2] < contact_band
    x_new[colliding, 2] = np.maximum(x_new[colliding, 2], 0.0)

    colliding_ids = np.where(colliding)[0]
    downward_ids = colliding_ids[v_new[colliding_ids, 2] < 0]
    v_new[downward_ids, 2] *= -0.1

    v_horiz = v_new[colliding, :2]
    speed = np.linalg.norm(v_horiz, axis=1, keepdims=True)
    drop = friction_coeff * dt
    mask = speed > drop
    v_new[colliding, :2] = np.where(mask, v_horiz * (1 - drop / speed), 0.0)

    return x_new, v_new


def mean_curvature_flow(
    x, v, faces, dt, gamma=0.5, rho=1.0, max_iterations=100, tol=1e-6
):
    """Apply mean curvature flow using implicit integration."""
    n_vertices = len(x)

    w = compute_cotangent_weights(x, faces)
    L = build_laplacian_matrix(n_vertices, faces, w)
    masses = compute_lumped_masses(x, faces, rho)
    M_inv = diags(1.0 / np.maximum(masses, 1e-8))

    I = sparse_eye(n_vertices)
    A = I + gamma * dt * M_inv @ L

    x_new = x.copy()
    for dim in range(3):
        b = x[:, dim]
        x_new[:, dim], info = cg(A, b, x0=x[:, dim], maxiter=max_iterations, rtol=tol)

    v_new = (x_new - x) / dt

    return x_new, v_new


def contact_angle_operator(x, v, faces, dt, alpha=0.5, contact_angles=None):
    """Apply contact angle constraint at ground level."""
    if contact_angles is None:
        receding_angle = np.deg2rad(70.0)
        advancing_angle = np.deg2rad(95.0)
    else:
        receding_angle, advancing_angle = contact_angles

    x_new = x.copy()
    v_new = v.copy()

    contact_band = 0.05
    contact_mask = x[:, 2] <= contact_band

    if np.any(contact_mask):
        n, _ = compute_vertex_normals_and_volume(x, faces)
        ground_normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        contact_ids = np.where(contact_mask)[0]
        for i in contact_ids:
            n_i = n[i]
            dot_val = np.clip(np.dot(n_i, ground_normal), -1.0, 1.0)
            angle = np.arccos(dot_val)

            n_p = n_i - dot_val * ground_normal
            n_p_norm = np.linalg.norm(n_p)

            if n_p_norm > 1e-6:
                n_p_dir = n_p / n_p_norm

                if angle < receding_angle:
                    f = alpha * (angle - receding_angle) * n_p_dir
                    x_new[i] += f * dt
                    v_new[i] += f / max(0.01, np.linalg.norm(v[i]) + 1e-8)
                elif angle > advancing_angle:
                    f = alpha * (angle - advancing_angle) * n_p_dir
                    x_new[i] += f * dt
                    v_new[i] += f / max(0.01, np.linalg.norm(v[i]) + 1e-8)

    return x_new, v_new


def apply_local_volume_correction(x, v, faces, rho=1.0):
    """Local volume correction to preserve details."""
    n, _ = compute_vertex_normals_and_volume(x, faces)
    w = compute_cotangent_weights(x, faces)

    masses = compute_lumped_masses(x, faces, rho)

    u = v.copy()

    a_i = np.sum(u * n, axis=1)

    neighbours = {i: set() for i in range(len(x))}
    for face in faces:
        for k in range(3):
            i = int(face[k])
            j = int(face[(k + 1) % 3])
            neighbours[i].add(j)
            neighbours[j].add(i)

    a_bar = np.zeros(len(x))
    for i in range(len(x)):
        area_sum = 0.0
        weighted_sum = 0.0
        for j in neighbours[i]:
            area = masses[j]
            weighted_sum += area * a_i[j]
            area_sum += area
        if area_sum > 1e-8:
            a_bar[i] = weighted_sum / area_sum

    u_corrected = u - a_bar[:, None] * n

    return u_corrected


def apply_global_volume_correction(x, faces, V_target, dt, volume_stiffness=1.0):
    """Global volume correction."""
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

    stiffness = np.clip(volume_stiffness, 0.0, 1.0)
    d = stiffness * (delta_V / total_area)

    x_corrected = x + d * n
    return x_corrected


def step_simulation(
    v,
    x,
    faces,
    neighbours,
    V_0,
    dt,
    method="euler",
    g=None,
    gamma=0.5,
    mu=0.3,
    eta=0.05,
    k_v=2000.0,
    density=1.0,
    rho=1.0,
    boundary_alpha=0.5,
    alpha=0.5,
    friction_coeff=0.5,
    receding_angle=np.deg2rad(70.0),
    advancing_angle=np.deg2rad(95.0),
    contact_angles=None,
    adhesion_dist=0.05,
    max_internal_accel=200.0,
    max_internal_substeps=8,
    volume_correction_mode="both",
):
    """Unified simulation step wrapper for explicit Euler integration only."""
    if g is None:
        g = np.array([0.0, 0.0, -9.81], dtype=np.float32)

    alpha_eff = alpha if alpha is not None else boundary_alpha
    density_eff = rho if rho is not None else density

    if contact_angles is not None:
        receding_eff, advancing_eff = contact_angles
    else:
        receding_eff, advancing_eff = receding_angle, advancing_angle

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
        boundary_alpha=alpha_eff,
        receding_angle=receding_eff,
        advancing_angle=advancing_eff,
        adhesion_dist=adhesion_dist,
        max_internal_accel=max_internal_accel,
        density=density_eff,
    )
