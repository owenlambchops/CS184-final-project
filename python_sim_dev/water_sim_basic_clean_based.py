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
    """generates a uniformly distributed set of points on the surface of a sphere using the Fibonacci method."""
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


def compute_lumped_areas(x, faces):
    """Compute lumped area for each vertex."""
    n_vertices = len(x)
    areas = np.zeros(n_vertices, dtype=np.float32)
    for f in faces:
        v0, v1, v2 = x[f[0]], x[f[1]], x[f[2]]
        face_area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
        areas[f] += face_area / 3.0
    return areas


def build_laplacian_matrix(x, faces):
    """Build discrete Laplace-Beltrami operator as sparse matrix."""
    row, col, data = [], [], []
    cotangent_weights = compute_cotangent_weights(x, faces)
    n_vertices = len(x)
    for i in range(n_vertices):
        row_sum = 0.0
        for j, w in cotangent_weights[i].items():
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


def apply_local_volume_correction(x, v, faces):
    n, _ = compute_vertex_normals_and_volume(x, faces)
    A = compute_lumped_areas(x, faces)
    vn = np.sum(v * n, axis=1)
    total_area = np.sum(A)
    if total_area < 1e-8:
        return v
    a_ave = np.sum(A * vn) / total_area
    return v - (n * a_ave)


def apply_global_volume_correction(x, faces, V_initial, fixed_mask=None):
    A = compute_lumped_areas(x, faces)
    if fixed_mask is None:
        fixed_mask = np.zeros(len(x), dtype=bool)
    movable = ~fixed_mask
    total_area = np.sum(A[movable])
    if total_area < 1e-8:
        return x
    # Solve global offset d = DeltaV/A iteratively to avoid overshooting.
    x_corr = x.copy()
    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edge_len = np.linalg.norm(x_corr[edges[:, 0]] - x_corr[edges[:, 1]], axis=1)
    d_max = 0.06 * max(np.mean(edge_len), 1e-5)
    for _ in range(4):
        n, V_current = compute_vertex_normals_and_volume(x_corr, faces)
        delta_V = V_initial - V_current
        if abs(delta_V) < 1e-9:
            break
        d = float(np.clip(delta_V / total_area, -d_max, d_max))
        x_corr[movable] += n[movable] * d
    return x_corr


def apply_implicit_mean_curvature_flow(x, faces, dt, gamma):
    """Paper Eq. (5): implicit mean-curvature-flow positional update."""
    if gamma <= 0.0:
        return x
    areas = compute_lumped_areas(x, faces)
    L = build_laplacian_matrix(x, faces)
    M = diags(areas)
    # Our L has negative diagonal / positive off-diagonal, so use M - gamma*dt*L.
    A = (M - (gamma * dt) * L).tocsr()
    x_new = np.zeros_like(x)
    for dim in range(3):
        b = areas * x[:, dim]
        sol, info = cg(A, b, rtol=1e-5, maxiter=200)
        if info != 0 or not np.all(np.isfinite(sol)):
            return x
        x_new[:, dim] = sol
    if not np.all(np.isfinite(x_new)):
        return x
    return x_new


def estimate_mesh_quality(faces, x):
    """Returns max_edge_len / mean_edge_len; higher means more stretched triangles."""
    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edge_len = np.linalg.norm(x[edges[:, 0]] - x[edges[:, 1]], axis=1)
    mean_len = np.mean(edge_len)
    if mean_len < 1e-8:
        return 1.0
    return float(np.max(edge_len) / mean_len)


def step(x, v, faces, neighbours, V_initial, dt):
    # 1. Physics Constants
    g = np.array([0.0, 0.0, -9.81], dtype=np.float32)
    gamma = 10.0
    mu = 2.0
    nu = 0.05
    nu_extra = 0.30
    max_speed = 20.0
    # Feed part of implicit positional correction back to velocity.
    # Without this, outward velocities can survive while positions are corrected,
    # causing repeated re-spreading in the next step.
    mcf_velocity_feedback = 0.05
    global_velocity_feedback = 0.0

    # 2. Adaptive internal substepping (stability under strong curvature/contact events)
    max_internal_substeps = 8
    safety = 0.25
    min_dt_safe = 1e-6

    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edge_len = np.linalg.norm(x[edges[:, 0]] - x[edges[:, 1]], axis=1)
    edge_ref = (
        float(np.median(edge_len[edge_len > 1e-8])) if np.any(edge_len > 1e-8) else 1e-3
    )

    # Probe explicit acceleration from gravity + viscosity only.
    # (MCF is implicit and displacement-capped below.)
    areas_probe = compute_lumped_areas(x, faces)
    L_probe = build_laplacian_matrix(x, faces)
    quality_probe = estimate_mesh_quality(faces, x)
    eta_probe = nu + (nu_extra if quality_probe > 2.6 else 0.0)
    inv_mass_probe = 1.0 / np.where(areas_probe > 1e-8, areas_probe, 1e-8)
    f_visc_probe = (L_probe @ v) * inv_mass_probe[:, np.newaxis] * eta_probe
    a_probe = g + f_visc_probe
    a_max = float(np.max(np.linalg.norm(a_probe, axis=1)))

    if a_max > 1e-8:
        dt_safe = safety * np.sqrt(edge_ref / a_max)
        internal_substeps = int(
            np.clip(np.ceil(dt / max(dt_safe, min_dt_safe)), 1, max_internal_substeps)
        )
    else:
        internal_substeps = 1

    dt_sub = dt / internal_substeps
    x_curr = x.copy()
    v_curr = v.copy()

    for _ in range(internal_substeps):
        # 3. Geometry
        areas = compute_lumped_areas(x_curr, faces)
        L = build_laplacian_matrix(x_curr, faces)
        quality_ratio = estimate_mesh_quality(faces, x_curr)
        eta = nu + (nu_extra if quality_ratio > 2.6 else 0.0)

        # 4. Forces and velocity integration
        inv_mass = 1.0 / np.where(areas > 1e-8, areas, 1e-8)
        f_visc = (L @ v_curr) * inv_mass[:, np.newaxis] * eta
        v_next = (v_curr + (g + f_visc) * dt_sub) * (1.0 - mu * dt_sub)

        # 5. Local volume correction
        v_trans = np.mean(v_next, axis=0)
        v_rel = v_next - v_trans
        v_rel = apply_local_volume_correction(x_curr, v_rel, faces)
        v_next = v_rel + v_trans

        # 6. Position + implicit MCF
        x_pred = x_curr + v_next * dt_sub
        x_next = apply_implicit_mean_curvature_flow(x_pred, faces, dt_sub, gamma)

        # tighter displacement cap
        edge_len_pred = np.linalg.norm(
            x_pred[edges[:, 0]] - x_pred[edges[:, 1]], axis=1
        )
        max_disp = 0.08 * max(np.median(edge_len_pred), 1e-5)
        disp = x_next - x_pred
        disp_len = np.linalg.norm(disp, axis=1)
        disp_scale = np.minimum(1.0, max_disp / np.maximum(disp_len, 1e-8))
        disp *= disp_scale[:, np.newaxis]
        x_next = x_pred + disp

        # Feed implicit MCF displacement back to velocity.
        if mcf_velocity_feedback > 0.0:
            v_next = v_next + mcf_velocity_feedback * (disp / max(dt_sub, 1e-8))

        # 7. Ground collision
        collision_mask = x_next[:, 2] < 0.0
        x_next[collision_mask, 2] = 0.0
        v_next[collision_mask, 2] = np.maximum(0, v_next[collision_mask, 2])
        v_next[collision_mask, :2] *= 0.5

        # 8. Global volume correction
        x_before_global = x_next.copy()
        x_next = apply_global_volume_correction(
            x_next, faces, V_initial, fixed_mask=collision_mask
        )
        x_next[collision_mask, 2] = 0.0

        # Also feed global correction back into velocity so constraints persist.
        if global_velocity_feedback > 0.0:
            v_next = v_next + global_velocity_feedback * (
                (x_next - x_before_global) / max(dt_sub, 1e-8)
            )

        # Final safety clamp per substep
        speed = np.linalg.norm(v_next, axis=1)
        speed_scale = np.minimum(1.0, max_speed / np.maximum(speed, 1e-8))
        v_next *= speed_scale[:, np.newaxis]

        if not np.all(np.isfinite(x_next)) or not np.all(np.isfinite(v_next)):
            return x.copy(), np.zeros_like(v)

        x_curr = x_next
        v_curr = v_next

    return x_curr, v_curr
