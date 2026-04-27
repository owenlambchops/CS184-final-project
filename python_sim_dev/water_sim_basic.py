import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from scipy.sparse.linalg import cg
from scipy.sparse import csr_matrix, eye as sparse_eye, diags
from collections import defaultdict


class WaterSim:
    def __init__(
        self,
        x,
        v,
        faces,
        neighbours,
        g=np.array([0.0, 0.0, -9.81], dtype=np.float32),
        gamma=0.5,
        mu=0.3,
        nu=0.05,
        k_v=2000.0,
        boundary_alpha=0.5,
        receding_angle=np.deg2rad(70.0),
        advancing_angle=np.deg2rad(95.0),
        adhesion_dist=0.05,
        max_internal_accel=200.0,
        max_internal_substeps=8,
        density=1.0,
        friction_coeff=0.5,
        nu_extra=0.5,
        mesh_quality_threshold=2.6,
    ):
        # Mesh State
        self.x = x
        self.v = v
        self.faces = faces
        self.neighbours = neighbours
        _, self.V_0 = self.compute_vertex_normals_and_volume(x)

        # Physics Parameters
        self.g = g
        self.gamma = gamma
        self.k_v = k_v
        self.boundary_alpha = boundary_alpha
        self.receding_angle = receding_angle
        self.advancing_angle = advancing_angle
        self.adhesion_dist = adhesion_dist
        self.max_internal_accel = max_internal_accel
        self.max_internal_substeps = max_internal_substeps
        self.density = density
        self.mu = mu
        self.nu =nu
        self.eta = nu
        self.nu_extra = nu_extra
        self.mesh_quality_threshold = mesh_quality_threshold
        self.friction_coeff = friction_coeff

    @staticmethod
    def normalise(vec):
        """Normalize a vector or array of vectors."""
        norm = np.linalg.norm(vec, axis=-1, keepdims=True)
        out = np.array(vec, copy=True)
        np.divide(vec, norm, out=out, where=norm > 0)
        return out

    @staticmethod
    def generate_points_on_sphere(n_pts=100, r=1.0):
        """Generates a uniformly distributed set of points on the surface of a sphere using the Fibonacci method."""
        goldenRatio = (1 + np.sqrt(5)) / 2
        idx = np.linspace(0, n_pts - 1, n_pts)
        theta = 2 * np.pi * idx / goldenRatio
        phi = np.arccos(1 - 2 * (idx) / n_pts)
        x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)
        return (np.stack((x, y, z), axis=1) * r).astype(np.float32)

    @staticmethod
    def generate_polyhedron(n_vertices=40, radius=1.0):
        """Generates a random convex polyhedron representing the initial droplet."""
        points = WaterSim.generate_points_on_sphere(n_pts=n_vertices, r=radius)
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

    @staticmethod
    def generate_rectangular_cube_mesh(
        width=1.0, height=1.0, length=1.0, nx=5, ny=5, nz=5, center=(0.0, 0.0, 0.0)
    ):
        """Generates a triangulated rectangular cuboid surface mesh."""
        if nx < 2 or ny < 2 or nz < 2:
            raise ValueError("nx, ny, nz must all be at least 2")

        cx, cy, cz = center
        xs = np.linspace(cx - width * 0.5, cx + width * 0.5, nx, dtype=np.float32)
        ys = np.linspace(cy - length * 0.5, cy + length * 0.5, ny, dtype=np.float32)
        zs = np.linspace(cz - height * 0.5, cz + height * 0.5, nz, dtype=np.float32)

        vertex_map = {}
        vertices = []

        def is_boundary(i, j, k):
            return (
                i == 0 or i == nx - 1 or j == 0 or j == ny - 1 or k == 0 or k == nz - 1
            )

        def vid(i, j, k):
            key = (i, j, k)
            if key in vertex_map:
                return vertex_map[key]
            if not is_boundary(i, j, k):
                raise ValueError("Requested an interior vertex for a surface mesh")
            vertex_map[key] = len(vertices)
            vertices.append((xs[i], ys[j], zs[k]))
            return vertex_map[key]

        face_list = []

        def add_quad(v00, v10, v11, v01, flip=False):
            if flip:
                face_list.append((v00, v11, v10))
                face_list.append((v00, v01, v11))
            else:
                face_list.append((v00, v10, v11))
                face_list.append((v00, v11, v01))

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
        faces = np.asarray(face_list, dtype=np.int32)

        neighbours = {i: set() for i in range(len(x))}
        for tri in faces:
            for k in range(3):
                i = int(tri[k])
                j = int(tri[(k + 1) % 3])
                neighbours[i].add(j)
                neighbours[j].add(i)

        return x, faces, {i: sorted(list(nbs)) for i, nbs in neighbours.items()}

    @staticmethod
    def generate_rectangle_mesh(
        width=1.0, height=1.0, nx=20, ny=20, center=(0.0, 0.0, 0.0)
    ):
        """Backward-compatible wrapper that generates a thin cuboid mesh."""
        thickness = min(width, height) * 0.05
        return WaterSim.generate_rectangular_cube_mesh(
            width=width,
            height=thickness,
            length=height,
            nx=nx,
            ny=ny,
            nz=2,
            center=center,
        )

    def compute_cotangent_weights(self, x):
        """Computes weights w_ij to approximate the Laplace-Beltrami operator"""
        w = defaultdict(dict)
        x0, x1, x2 = x[self.faces[:, 0]], x[self.faces[:, 1]], x[self.faces[:, 2]]

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

        for f, (i0, i1, i2) in enumerate(self.faces):
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

    def compute_vertex_normals_and_volume(self, x):
        """Calculates vertex normals (n_i) and the exact internal volume (V) of the mesh."""
        n = np.zeros_like(x)
        V = 0.0
        com = np.mean(x, axis=0)

        for i0, i1, i2 in self.faces:
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

    def compute_lumped_masses(self, x):
        """Compute lumped mass matrix for triangular mesh."""
        return self.compute_lumped_areas(x) * self.density

    def compute_lumped_areas(self, x):
        """Compute lumped area matrix for triangular mesh."""
        n_vertices = len(x)
        areas = np.zeros(n_vertices, dtype=np.float32)

        for i in range(n_vertices):
            vertex_area = 0.0
            for face in self.faces:
                if i in face:
                    a, b, c = x[face[0]], x[face[1]], x[face[2]]
                    vertex_area += 0.5 * np.linalg.norm(np.cross(b - a, c - a)) / 3.0
            areas[i] = vertex_area

        return areas

    def build_laplacian_matrix(self, n_vertices, cotangent_weights):
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

    def compute_laplacians(self, x, v, w):
        """Computes positional (Delta x) and velocity (Delta v) laplacians for all vertices."""
        delta_x = np.zeros_like(x)
        delta_v = np.zeros_like(v)

        for i in range(len(x)):
            sum_w = 0.0
            for j in self.neighbours[i]:
                w_ij = w[i].get(j, 1.0)
                delta_x[i] += w_ij * (x[j] - x[i])
                delta_v[i] += w_ij * (v[j] - v[i])
                sum_w += w_ij

            if sum_w > 0:
                delta_x[i] /= sum_w
                delta_v[i] /= sum_w

        return delta_x, delta_v

    def apply_surface_interactions(self, x, v, dt):
        """Handles Z=0 plane collisions, boundary constraints, and sliding friction."""
        x = x.copy()
        v = v.copy()

        for i in range(len(x)):
            if x[i][2] <= self.adhesion_dist:
                if x[i][2] < 0:
                    x[i][2] = 0
                    if v[i][2] < 0:
                        v[i][2] *= -0.2

                v_horiz = v[i][:2]
                speed = np.linalg.norm(v_horiz)
                if speed > 0:
                    drop = self.friction_coeff * dt
                    if speed < drop:
                        v[i][:2] = 0.0
                    else:
                        v[i][:2] *= (speed - drop) / speed

                v[i][:2] = (1 - 0.05 * dt) * v[i][:2]

        return x, v

    def boundary_force(
        self,
        n_l,
        contact_mask,
        ground_normal=np.array([0.0, 0.0, 1.0], dtype=np.float32),
    ):
        """Computes per-vertex contact-angle hysteresis boundary force in radians."""
        if self.receding_angle > self.advancing_angle:
            raise ValueError("receding_angle must be <= advancing_angle")

        f_boundary = np.zeros_like(n_l)
        if np.count_nonzero(contact_mask) == 0:
            return f_boundary

        n_i = self.normalise(ground_normal.astype(n_l.dtype, copy=False))
        contact_ids = np.where(contact_mask)[0]

        for i in contact_ids:
            n_li = n_l[i]
            dot_val = np.clip(np.dot(n_li, n_i), -1.0, 1.0)
            angle = np.arccos(dot_val)

            n_p = n_li - dot_val * n_i
            n_p_dir = self.normalise(n_p)

            if np.linalg.norm(n_p_dir) == 0:
                continue

            if self.receding_angle < angle < self.advancing_angle:
                continue
            if angle <= self.receding_angle:
                f_boundary[i] = (
                    self.boundary_alpha * (angle - self.receding_angle) * n_p_dir
                )
            else:
                f_boundary[i] = (
                    self.boundary_alpha * (angle - self.advancing_angle) * n_p_dir
                )

        return f_boundary

    def compute_total_acceleration(self, x, v):
        """Computes total acceleration and velocity Laplacian for a given state."""
        w = self.compute_cotangent_weights(x)
        n, V = self.compute_vertex_normals_and_volume(x)
        delta_x, delta_v = self.compute_laplacians(x, v, w)

        f_st = self.gamma * delta_x
        f_vol = self.k_v * (self.V_0 - V) * n
        contact_mask = x[:, 2] <= self.adhesion_dist
        f_boundary = self.boundary_force(n, contact_mask)

        a_ext = np.broadcast_to(self.g, x.shape)
        a_internal = (f_st + f_vol + f_boundary) / self.density

        a_int_norm = np.linalg.norm(a_internal, axis=1, keepdims=True)
        scale = np.minimum(1.0, self.max_internal_accel / np.maximum(a_int_norm, 1e-8))
        a_internal = a_internal * scale

        a_total = a_ext + a_internal
        a_total = np.nan_to_num(a_total, nan=0.0, posinf=0.0, neginf=0.0)
        delta_v = np.nan_to_num(delta_v, nan=0.0, posinf=0.0, neginf=0.0)
        return a_total, delta_v

    def compute_reference_edge_length(self, x):
        """Returns a robust characteristic edge length for adaptive time stepping."""
        if len(self.faces) == 0:
            return 1e-3

        e01 = np.linalg.norm(x[self.faces[:, 0]] - x[self.faces[:, 1]], axis=1)
        e12 = np.linalg.norm(x[self.faces[:, 1]] - x[self.faces[:, 2]], axis=1)
        e20 = np.linalg.norm(x[self.faces[:, 2]] - x[self.faces[:, 0]], axis=1)
        edges = np.concatenate((e01, e12, e20))
        edges = edges[edges > 1e-8]
        if edges.size == 0:
            return 1e-3
        return float(np.median(edges))

    def estimate_mesh_quality(self, x):
        """Returns max_edge_len / mean_edge_len; higher means more stretched triangles."""
        if len(self.faces) == 0:
            return 1.0
        edges = np.vstack(
            (self.faces[:, [0, 1]], self.faces[:, [1, 2]], self.faces[:, [2, 0]])
        )
        edge_len = np.linalg.norm(x[edges[:, 0]] - x[edges[:, 1]], axis=1)
        mean_len = np.mean(edge_len)
        if mean_len < 1e-8:
            return 1.0
        return float(np.max(edge_len) / mean_len)

    def compute_dynamic_eta(self, x):
        """Increase viscosity compensation when the mesh quality degrades."""
        quality = self.estimate_mesh_quality(x)
        if quality <= self.mesh_quality_threshold:
            return self.eta
        excess = (quality - self.mesh_quality_threshold) / max(
            self.mesh_quality_threshold, 1e-8
        )
        self.eta = self.nu + self.nu_extra * min(1.0, excess)
        return self.eta

    def step_forward_euler(self, dt):
        """Advances the simulation by one explicit Euler time step."""
        a_total, delta_v = self.compute_total_acceleration(self.x, self.v)
        eta_eff = self.compute_dynamic_eta(self.x)

        v_damped = (self.v + (eta_eff * dt * delta_v)) / (1.0 + self.mu * dt)
        v_new = np.clip(v_damped + a_total * dt, -15.0, 15.0)
        x_new = self.x + v_new * dt
        x_new, v_new = self.apply_surface_interactions(x_new, v_new, dt)

        self.x, self.v = x_new, v_new
        return self.v, self.x

    def step_forward_verlet(self, dt):
        """Advances the simulation by one explicit Verlet time step."""
        a_total, delta_v = self.compute_total_acceleration(self.x, self.v)
        eta_eff = self.compute_dynamic_eta(self.x)

        v_damped = (self.v + (eta_eff * dt * delta_v)) / (1.0 + self.mu * dt)
        v_new = v_damped + a_total * dt
        # x_new = self.x + 0.5 * dt * (self.v + v_new)
        x_new = self.x + v_new * dt + dt * dt * 0.5 * a_total
        x_new, v_new = self.apply_surface_interactions(x_new, v_new, dt)

        self.x, self.v = x_new, v_new
        return self.v, self.x

    def step_verlet(self, dt):
        """Advances the simulation by one velocity-Verlet time step."""
        edge_len = self.compute_reference_edge_length(self.x)
        a_probe, _ = self.compute_total_acceleration(self.x, self.v)

        a_max = float(np.max(np.linalg.norm(a_probe, axis=1)))
        if a_max > 1e-8:
            dt_safe = 0.35 * np.sqrt(edge_len / a_max)
            internal_substeps = int(
                np.clip(np.ceil(dt / max(dt_safe, 1e-6)), 1, self.max_internal_substeps)
            )
        else:
            internal_substeps = 1

        dt_sub = dt / internal_substeps
        x_curr = self.x.copy()
        v_curr = self.v.copy()

        for _ in range(internal_substeps):
            a_n, delta_v_n = self.compute_total_acceleration(x_curr, v_curr)
            eta_eff = self.compute_dynamic_eta(x_curr)

            damp_half = 1.0 + 0.5 * self.mu * dt_sub
            v_half = (
                v_curr + 0.5 * dt_sub * a_n + 0.5 * eta_eff * dt_sub * delta_v_n
            ) / damp_half
            v_half = np.nan_to_num(v_half, nan=0.0, posinf=0.0, neginf=0.0)

            x_next = x_curr + dt_sub * v_half
            x_next = np.nan_to_num(x_next, nan=0.0, posinf=0.0, neginf=0.0)

            a_np1, delta_v_np1 = self.compute_total_acceleration(x_next, v_half)

            v_next = (
                v_half + 0.5 * dt_sub * a_np1 + 0.5 * eta_eff * dt_sub * delta_v_np1
            ) / damp_half
            v_next = np.clip(
                np.nan_to_num(v_next, nan=0.0, posinf=0.0, neginf=0.0), -15.0, 15.0
            )

            x_next, v_next = self.apply_surface_interactions(x_next, v_next, dt_sub)

            x_curr = x_next
            v_curr = v_next

        self.x, self.v = x_curr, v_curr
        return self.v, self.x

    def apply_external_forces(self, dt):
        """Apply gravity, collision, and friction directly to class state."""
        x_new = self.x.copy()
        v_new = self.v.copy()

        v_new = v_new + self.g * dt
        x_new = x_new + v_new * dt

        contact_band = 0.05
        colliding = x_new[:, 2] < contact_band
        x_new[colliding, 2] = np.maximum(x_new[colliding, 2], 0.0)

        colliding_ids = np.where(colliding)[0]
        downward_ids = colliding_ids[v_new[colliding_ids, 2] < 0]
        v_new[downward_ids, 2] *= -0.1

        v_horiz = v_new[colliding, :2]
        speed = np.linalg.norm(v_horiz, axis=1, keepdims=True)
        drop = self.friction_coeff * dt
        mask = speed > drop
        v_new[colliding, :2] = np.where(mask, v_horiz * (1 - drop / speed), 0.0)

        self.x, self.v = x_new, v_new
        return self.x, self.v

    def mean_curvature_flow(self, dt, max_iterations=100, tol=1e-6):
        """Apply mean curvature flow using implicit integration on current state."""
        n_vertices = len(self.x)

        w = self.compute_cotangent_weights(self.x)
        L = self.build_laplacian_matrix(n_vertices, w)
        masses = self.compute_lumped_masses(self.x)
        M_inv = diags(1.0 / np.maximum(masses, 1e-8))

        I = sparse_eye(n_vertices)
        A = I + self.gamma * dt * M_inv @ L

        x_new = self.x.copy()
        for dim in range(3):
            b = self.x[:, dim]
            x_new[:, dim], _ = cg(
                A, b, x0=self.x[:, dim], maxiter=max_iterations, rtol=tol
            )

        v_new = (x_new - self.x) / dt

        self.x, self.v = x_new, v_new
        return self.x, self.v

    def contact_angle_operator(self, dt):
        """Apply contact angle constraint at ground level."""
        x_new = self.x.copy()
        v_new = self.v.copy()

        contact_band = 0.05
        contact_mask = self.x[:, 2] <= contact_band

        if np.any(contact_mask):
            n, _ = self.compute_vertex_normals_and_volume(self.x)
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

                    if angle < self.receding_angle:
                        f = (
                            self.boundary_alpha
                            * (angle - self.receding_angle)
                            * n_p_dir
                        )
                        x_new[i] += f * dt
                        v_new[i] += f / max(0.01, np.linalg.norm(self.v[i]) + 1e-8)
                    elif angle > self.advancing_angle:
                        f = (
                            self.boundary_alpha
                            * (angle - self.advancing_angle)
                            * n_p_dir
                        )
                        x_new[i] += f * dt
                        v_new[i] += f / max(0.01, np.linalg.norm(self.v[i]) + 1e-8)

        self.x, self.v = x_new, v_new
        return self.x, self.v

    def apply_local_volume_correction(self, velocity=None):
        """Remove area-weighted mean normal velocity from a velocity field."""
        if velocity is None:
            velocity = self.v
            update_state = True
        else:
            update_state = False

        n, _ = self.compute_vertex_normals_and_volume(self.x)
        areas = self.compute_lumped_areas(self.x)
        vn = np.sum(velocity * n, axis=1)
        total_area = np.sum(areas)
        if total_area < 1e-8:
            return velocity
        a_ave = np.sum(areas * vn) / total_area
        corrected = velocity - (n * a_ave)
        if update_state:
            self.v = corrected
        return corrected

    def apply_global_volume_correction(
        self,
        ground_mask=None,
        radial_spread=0.0,
        iterations=4,
        volume_stiffness=0.2,
    ):
        """Correct global volume by moving vertices along normals with safeguards."""
        areas = self.compute_lumped_areas(self.x)
        total_area = np.sum(areas)
        if total_area < 1e-8:
            return self.x

        x_corr = self.x.copy()
        edges = np.vstack(
            (self.faces[:, [0, 1]], self.faces[:, [1, 2]], self.faces[:, [2, 0]])
        )
        edge_len = np.linalg.norm(x_corr[edges[:, 0]] - x_corr[edges[:, 1]], axis=1)
        d_max = 0.15 * max(np.mean(edge_len), 1e-5)
        stiffness = np.clip(volume_stiffness, 0.0, 1.0)

        for _ in range(iterations):
            n, v_current = self.compute_vertex_normals_and_volume(x_corr)
            delta_v = self.V_0 - v_current
            if abs(delta_v) < 1e-9:
                break

            d = stiffness * (delta_v / total_area)
            d = np.clip(d, -d_max, d_max)
            disp = d * n

            if ground_mask is not None and np.any(ground_mask):
                com = np.mean(x_corr, axis=0)
                horiz = disp[ground_mask].copy()
                horiz[:, 2] = 0.0
                if radial_spread > 0.0:
                    pos = x_corr[ground_mask]
                    r = pos - com
                    r[:, 2] = 0.0
                    r_norm = np.linalg.norm(r, axis=1, keepdims=True)
                    r_dir = np.divide(
                        r, r_norm, out=np.zeros_like(r), where=r_norm > 1e-8
                    )
                    horiz = 0.5 * horiz + 0.5 * radial_spread * d * r_dir
                disp[ground_mask] = horiz

            x_corr = x_corr + disp

        self.x = x_corr
        return self.x

    def step_simulation(self, dt, method="euler", volume_correction_mode="both"):
        """Unified simulation step wrapper prioritizing configured properties."""
        if method == "euler":
            self.step_forward_euler(dt)
        elif method == "verlet":
            # return self.step_verlet(dt)
            self.step_forward_verlet(dt)
        else:
            raise ValueError(f"Unknown integration method: {method}")

        # Scale global correction for small droplets to avoid over-correction.
        volume_scale = np.clip(self.V_0 / 1e-2, 0.1, 1.0)
        ground_mask = self.x[:, 2] <= self.adhesion_dist

        # Apply requested volume correction after the integration step.
        if volume_correction_mode == "local":
            v_trans = np.mean(self.v, axis=0)
            v_rel = self.v - v_trans
            self.v = self.apply_local_volume_correction(velocity=v_rel) + v_trans
        elif volume_correction_mode == "global":
            self.apply_global_volume_correction(
                ground_mask=ground_mask,
                radial_spread=0.0,
                volume_stiffness=0.2 * volume_scale,
            )
            self.x, self.v = self.apply_surface_interactions(self.x, self.v, dt)
        elif volume_correction_mode == "both":
            v_trans = np.mean(self.v, axis=0)
            v_rel = self.v - v_trans
            self.v = self.apply_local_volume_correction(velocity=v_rel) + v_trans
            self.apply_global_volume_correction(
                ground_mask=ground_mask,
                radial_spread=0.0,
                volume_stiffness=0.2 * volume_scale,
            )
            self.x, self.v = self.apply_surface_interactions(self.x, self.v, dt)
        elif volume_correction_mode != "none":
            raise ValueError(
                "volume_correction_mode must be one of {'none', 'local', 'global', 'both'}"
            )

        return self.v, self.x
