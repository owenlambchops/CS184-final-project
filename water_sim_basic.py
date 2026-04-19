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


def step(
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
    adhesion_dist=0.05,
    density=1.0,
):  # <--- ADDED DENSITY PARAMETER
    """Advances the simulation by one explicit Euler time step."""

    # 1. Update State Geometry
    w = compute_cotangent_weights(x, faces)
    n, V = compute_vertex_normals_and_volume(x, faces)
    delta_x, delta_v = compute_laplacians(x, v, neighbours, w)

    # 2. Accumulate Forces
    f_st = gamma * delta_x  # Surface Tension Force
    f_vol = k_v * (V_0 - V) * n  # Volume Pressure Force

    # 3. Apply Density (Newton's Second Law: a = F / m)
    # Gravity is an acceleration, so it is independent of mass.
    # Internal forces are divided by the mass/density of the vertices.
    a_ext = np.broadcast_to(g, x.shape)
    a_internal = (f_st + f_vol) / density

    a_total = a_ext + a_internal

    # 4. Time Integration
    v_damped = (1.0 - mu * dt) * v + (eta * dt * delta_v)

    # Update velocity using the newly scaled acceleration
    v_new = v_damped + a_total * dt
    v_new = np.clip(v_new, -15.0, 15.0)
    x_new = x + v_new * dt

    # 5. Apply Environmental Constraints (Glass surface)
    x_new, v_new = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, adhesion_dist
    )

    return v_new, x_new
