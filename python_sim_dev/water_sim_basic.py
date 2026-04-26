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
    return np.where(norm > 0, vec / norm, vec)


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
    """Calculates vertex normals (n_i) and the exact internal volume (V)."""
    n = np.zeros_like(x)
    V = 0.0
    com = np.mean(x, axis=0)
    for i0, i1, i2 in faces:
        x0, x1, x2 = x[i0], x[i1], x[i2]
        n_face = np.cross(x1 - x0, x2 - x0)
        face_center = (x0 + x1 + x2) / 3.0
        # ConvexHull.simplices does not guarantee a consistent winding, so
        # re-orient each face normal before using it for vertex normals/volume.
        if np.dot(n_face, face_center - com) < 0.0:
            n_face = -n_face
        n[i0] += n_face
        n[i1] += n_face
        n[i2] += n_face
        V += np.dot(x0 - com, n_face) / 6.0 # Corrected volume formula
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
    d_max = 0.15 * max(np.mean(edge_len), 1e-5)
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
    gamma = 0.05  # Surface tension strength for implicit mean curvature flow
    mu = 0.2     # Global damping
    nu = 0.05    # Velocity laplacian (viscosity)
    nu_extra = 0.30  # Extra viscosity when mesh quality degrades
    max_speed = 20.0  # Stability guard against runaway integration

    # 2. Geometry
    areas = compute_lumped_areas(x, faces)
    L = build_laplacian_matrix(x, faces)
    quality_ratio = estimate_mesh_quality(faces, x)
    # Paper Sec. 4.1: use larger eta when mesh quality is unsatisfactory.
    eta = nu + (nu_extra if quality_ratio > 2.6 else 0.0)

    # 3. Forces
    # Accel_st = (gamma * L @ x) / mass. Here we use areas as proxy for mass.
    inv_mass = 1.0 / np.where(areas > 1e-8, areas, 1e-8)
    
    # Viscosity acceleration (Velocity smoothing)
    f_visc = (L @ v) * inv_mass[:, np.newaxis] * eta

    # 4. Integrate Velocity (external forces + viscosity damping, paper Eq. (3)-style)
    v_next = (v + (g + f_visc) * dt) * (1.0 - mu * dt)
    speed = np.linalg.norm(v_next, axis=1)
    speed_scale = np.minimum(1.0, max_speed / np.maximum(speed, 1e-8))
    v_next *= speed_scale[:, np.newaxis]

    # 5. Volume Preservation Constraints
    v_trans = np.mean(v_next, axis=0)
    v_rel = v_next - v_trans
    v_rel = apply_local_volume_correction(x, v_rel, faces)
    v_next = v_rel + v_trans
    
    # 6. Update Position and apply implicit mean curvature flow (paper Eq. (5))
    x_pred = x + v_next * dt
    x_next = apply_implicit_mean_curvature_flow(x_pred, faces, dt, gamma)
    # Add the implicit surface-tension displacement back into velocity.
    edges = np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))
    edge_len = np.linalg.norm(x_pred[edges[:, 0]] - x_pred[edges[:, 1]], axis=1)
    max_disp = 0.25 * max(np.mean(edge_len), 1e-5)
    disp = x_next - x_pred
    disp_len = np.linalg.norm(disp, axis=1)
    disp_scale = np.minimum(1.0, max_disp / np.maximum(disp_len, 1e-8))
    disp *= disp_scale[:, np.newaxis]
    x_next = x_pred + disp
    v_next = v_next + disp / dt

    # 7. Ground Collision
    collision_mask = x_next[:, 2] < 0.0
    x_next[collision_mask, 2] = 0.0
    v_next[collision_mask, 2] = np.maximum(0, v_next[collision_mask, 2])
    v_next[collision_mask, :2] *= 0.5 
    
    # 8. Paper Sec. 4.4 global correction as normal-direction positional offset.
    x_next = apply_global_volume_correction(x_next, faces, V_initial, fixed_mask=collision_mask)
    x_next[collision_mask, 2] = 0.0
    if not np.all(np.isfinite(x_next)) or not np.all(np.isfinite(v_next)):
        return x.copy(), np.zeros_like(v)

    return x_next, v_next
