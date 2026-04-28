# -*- coding: utf-8 -*-
"""
Real-Time Interactive Water Droplets (Euler & Advanced Damping)
Simulates a deformable water droplet on a surface using Vertex Dynamics.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from collections import defaultdict


def normalise(vec):
    """Normalize a vector or array of vectors."""
    norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    return np.where(norm > 0, vec / norm, vec)


def generate_points_on_sphere(n_pts=100, r=1.0):
    """Generates a uniformly distributed set of points on the surface of a sphere."""
    """https://extremelearning.com.au/how-to-evenly-distribute-points-on-a-sphere-more-effectively-than-the-canonical-fibonacci-lattice/"""
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
    x_new = x.copy()
    v_new = v.copy()

    for i in range(len(x_new)):
        if x_new[i][2] <= adhesion_dist:
            if x_new[i][2] < 0:
                x_new[i][2] = 0
                if v_new[i][2] < 0:
                    v_new[i][2] *= -0.2

            v_horiz = v_new[i][:2]
            speed = np.linalg.norm(v_horiz)
            if speed > 0:
                drop = friction_coeff * dt
                if speed < drop:
                    v_new[i][:2] = 0.0
                else:
                    v_new[i][:2] *= (speed - drop) / speed

            # Additional surface slip drag
            v_new[i][:2] = (1 - 0.05 * dt) * v_new[i][:2]

    return x_new, v_new


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


def compute_accelerations(
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


def update_forward_euler(
    a,
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
    damping_gain=1.0,
):
    """Advances the simulation by one explicit Euler time step."""
    a_total, delta_v = compute_accelerations(
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

    # Base viscosity and regular damping combination
    v_damped = (1.0 - mu * dt) * v + (eta * dt) * delta_v

    # Energy-aware damping: linear damping + quadratic drag to kill oscillations over time.
    speed = np.linalg.norm(v, axis=1, keepdims=True)
    lin = np.maximum(0.0, mu) * float(damping_gain)
    quad = (0.2 + 0.3 * np.maximum(0.0, eta)) * float(damping_gain)
    damping = 1.0 / (1.0 + lin * dt + quad * speed * dt)

    # Apply the computed damping scaling to v_damped
    v_damped = v_damped * damping

    # Residual global damping in low-speed regime to ensure eventual settling.
    mean_speed = float(np.mean(speed))
    if mean_speed < 0.25:
        v_damped *= max(0.0, 1.0 - 0.3 * dt * float(damping_gain))

    v_new = np.clip(v_damped + a_total * dt, -15.0, 15.0)
    x_new = x + v_new * dt

    x_new, v_new = apply_surface_interactions(
        x_new, v_new, dt, friction_coeff, adhesion_dist
    )

    return a_total, v_new, x_new


def update_velocity_verlet(
    a,
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
    damping_gain=1.0,
):
    """Advances the simulation by one explicit Velocity Verlet time step."""
    """https://en.wikipedia.org/wiki/Verlet_integration#Algorithmic_representation"""
    # Velocity Verlet integration scheme:
    # x_new = x + v*dt + a*(dt²/2)
    # a_new = compute_accelerations(x_new)
    # v_new = v + (a + a_new)*(dt/2)

    x_new = x + v * dt + a * (dt * dt * 0.5)
    a_new, delta_v = compute_accelerations(
        x_new,
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

    v_new = v + (a + a_new) * (dt * 0.5)
    v_damped = (1.0 - mu * dt) * v_new + (eta * dt) * delta_v

    # Energy-aware damping: linear damping + quadratic drag to kill oscillations over time.
    speed = np.linalg.norm(v_new, axis=1, keepdims=True)
    lin = np.maximum(0.0, mu) * float(damping_gain)
    quad = (0.2 + 0.3 * np.maximum(0.0, eta)) * float(damping_gain)
    damping = 1.0 / (1.0 + lin * dt + quad * speed * dt)

    # Apply the computed damping scaling to v_damped
    v_damped = v_damped * damping

    # Residual global damping in low-speed regime to ensure eventual settling.
    mean_speed = float(np.mean(speed))
    if mean_speed < 0.25:
        v_damped *= max(0.0, 1.0 - 0.3 * dt * float(damping_gain))

    # Clip velocities to prevent simulation instability
    v_damped = np.clip(v_damped, -15.0, 15.0)

    # Apply surface interactions with the damped velocity
    x_new, v_final = apply_surface_interactions(
        x_new, v_damped, dt, friction_coeff, adhesion_dist
    )

    return a_new, v_final, x_new


def update(
    a,
    v,
    x,
    faces,
    neighbours,
    V_0,
    dt,
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
    damping_gain=1.0,
    method="euler",
):
    """Unified simulation step wrapper for euler mode."""
    if g is None:
        g = np.array([0.0, 0.0, -9.81], dtype=np.float32)

    alpha_eff = alpha if alpha is not None else boundary_alpha
    density_eff = rho if rho is not None else density

    if contact_angles is not None:
        receding_eff, advancing_eff = contact_angles
    else:
        receding_eff, advancing_eff = receding_angle, advancing_angle

    if method == "verlet":
        return update_velocity_verlet(
            a,
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
            damping_gain=damping_gain,
        )
    
    return update_forward_euler(
        a,
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
        damping_gain=damping_gain,
    )
