import copy
import numpy as np
from scipy.spatial import ConvexHull


def normalise(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def L_umbrella(i, positions, neighbours):
    """approximation of the Laplacian, Fujiwara 1995
    Desbrun et al. 1999"""
    L = 0.0
    E = 0.0

    x_i = positions[i, :]
    for j in neighbours[i]:
        x_j = positions[j, :]
        L += normalise(x_i - x_j)
        E += np.linalg.norm(x_i - x_j)

    return L * 2 / E if E > 0 else np.zeros_like(x_i)


def L(n, cotangent_weights, neighbours):
    """
    Build the symmetric Laplacian matrix L using cotangent weights.
    Args:
        n (int): Number of vertices.
        cotangent_weights (dict of dict): weights[i][j] = cotangent weight for edge (i, j)
        neighbours (dict): neighbours[i] = list of neighbor indices for vertex i
    Returns:
        L (np.ndarray): (n, n) Laplacian matrix.
    """
    L_ret = np.zeros((n, n), dtype=np.float32)
    for i in range(n):
        row_sum = 0.0
        for j in neighbours[i]:
            w = cotangent_weights[i][j] if j in cotangent_weights[i] else 0.0
            L_ret[i, j] = w
            row_sum += w
        L_ret[i, i] = -row_sum
    return L_ret


def M(n, rho, positions, faces):
    """
    Build a lumped mass matrix for a triangular surface mesh.

    Each vertex gets one third of the area of every adjacent triangle, scaled by
    the fluid density rho. The result is a diagonal matrix where M[i, i] is the
    mass associated with vertex i.

    Args:
        n (int): Number of vertices.
        rho (float): Fluid density.
        positions (np.ndarray): (n, 3) array of vertex positions.
        faces (np.ndarray): (f, 3) array of triangle indices.

    Returns:
        np.ndarray: (n, n) diagonal lumped mass matrix.
    """

    def triangle_area(a, b, c):
        return 0.5 * np.linalg.norm(np.cross(b - a, c - a))

    mass_matrix = np.zeros((n, n), dtype=np.float32)

    for i in range(n):
        vertex_mass = 0.0
        for face in faces:
            if i in face:
                a = positions[face[0]]
                b = positions[face[1]]
                c = positions[face[2]]
                vertex_mass += triangle_area(a, b, c) / 3.0
        mass_matrix[i, i] = rho * vertex_mass

    return mass_matrix


def generate_points_on_sphere(n=100, r=1.0):
    """
    Generate n points approximately evenly distributed on the surface of a sphere of radius r.

    Args:
        n (int): Number of points to generate. Output will have shape (n, 3).
        r (float): Radius of the sphere.

    Returns:
        points (np.ndarray): Array of shape (n, 3), where each row (dimension 0) is the (x, y, z) coordinates of a point on the sphere.
    """
    idx = np.linspace(0, n - 1, n)

    y = 1 - (idx / float(n - 1)) * 2  # y goes from 1 to -1
    radius = np.sqrt(1 - y * y)  # radius at y
    phi = idx * 3.6  # golden angle increment
    x = np.cos(phi) * radius
    z = np.sin(phi) * radius
    points = np.stack((x, y, z), axis=1) * r
    return points.astype(np.float32)


def generate_polyhedron(n_vertices=8, radius=1.0):
    """
    Generate a random convex polyhedron using n random points on the sphere.
    Returns (vertices, faces, neighbours) in the same format as before.
    """

    # Generate n random points on the sphere
    points = generate_points_on_sphere(n=n_vertices, r=radius)
    hull = ConvexHull(points)
    vertices = points.astype(np.float32)
    faces = hull.simplices  # Each row is a triangle (3 indices)
    # Build neighbours from faces
    neighbours = {i: set() for i in range(len(vertices))}
    for tri in faces:
        for k in range(3):
            i = tri[k]
            j = tri[(k + 1) % 3]
            neighbours[i].add(j)
            neighbours[j].add(i)
    # Convert sets to sorted lists
    neighbours = {i: sorted(list(nbs)) for i, nbs in neighbours.items()}
    return vertices, faces, neighbours


def compute_cotangent_weights(V, F):
    """
    Compute cotangent weights for each edge in a triangular mesh, following the discrete Laplace-Beltrami operator formulation:

        l_ij = -0.5 * (cot(alpha_ij) + cot(beta_ij)), for j in N(i)
        l_ii = -sum_{j in N(i)} l_ij

    where alpha_ij and beta_ij are the two angles opposite edge (i, j).

    Reference:
        - "Discrete Differential-Geometry Operators for Triangulated 2-Manifolds" (Meyer, Desbrun, Schröder, Barr, 2003)
        - Section 3.2, Cotangent Formula

    Args:
        V (np.ndarray): (n, 3) array of vertex positions.
        F (np.ndarray): (f, 3) array of triangle indices.

    Returns:
        L (dict of dict): L[i][j] = l_ij cotangent weight for edge (i, j)
    """
    from collections import defaultdict
    import numpy as np

    L = defaultdict(dict)

    # For each triangle, get its three vertices
    v0 = V[F[:, 0]]
    v1 = V[F[:, 1]]
    v2 = V[F[:, 2]]

    # Compute cotangent of each angle in the triangle using vector math
    # cotangent at v0 (opposite edge v1-v2)
    cot_alpha = np.einsum("ij,ij->i", v1 - v0, v2 - v0) / np.linalg.norm(
        np.cross(v1 - v0, v2 - v0), axis=1
    )
    # cotangent at v1 (opposite edge v2-v0)
    cot_beta = np.einsum("ij,ij->i", v2 - v1, v0 - v1) / np.linalg.norm(
        np.cross(v2 - v1, v0 - v1), axis=1
    )
    # cotangent at v2 (opposite edge v0-v1)
    cot_gamma = np.einsum("ij,ij->i", v0 - v2, v1 - v2) / np.linalg.norm(
        np.cross(v0 - v2, v1 - v2), axis=1
    )

    # Replace any nan or inf with 0 for numerical stability
    cot_alpha = np.nan_to_num(cot_alpha)
    cot_beta = np.nan_to_num(cot_beta)
    cot_gamma = np.nan_to_num(cot_gamma)

    # For each triangle, assign cotangent weights to each of its three edges
    for f, face in enumerate(F):
        i, j, k = face
        # Edge (i, j), opposite angle at k
        l_ij = -0.5 * (cot_gamma[f])
        L[i][j] = L[i].get(j, 0.0) + l_ij
        L[j][i] = L[j].get(i, 0.0) + l_ij
        # Edge (j, k), opposite angle at i
        l_jk = -0.5 * (cot_alpha[f])
        L[j][k] = L[j].get(k, 0.0) + l_jk
        L[k][j] = L[k].get(j, 0.0) + l_jk
        # Edge (k, i), opposite angle at j
        l_ki = -0.5 * (cot_beta[f])
        L[k][i] = L[k].get(i, 0.0) + l_ki
        L[i][k] = L[i].get(k, 0.0) + l_ki

    return L


def apply_gravity(positions, velocities, g, dt):
    return positions, velocities + g * dt


def apply_collision_and_friction(positions, velocities, friction_coeff):
    new_velocities = velocities.copy()
    new_positions = positions.copy()
    contact_band = 0.02
    grounded = new_positions[:, 2] <= contact_band
    if np.any(grounded):
        # Let the bottom ring stick to the plane while the upper vertices keep moving.
        new_positions[grounded, 2] = 0.0

        grounded_indices = np.where(grounded)[0]
        downward_velocity = new_velocities[grounded_indices, 2] < 0
        new_velocities[grounded_indices[downward_velocity], 2] = 0.0

        damping = max(0.0, 1.0 - 0.5 * friction_coeff)
        new_velocities[grounded, :2] *= damping
    return new_positions, new_velocities


def apply_viscosity(
    positions, velocities, neighbours, cotangent_weights, dt, mu, nabla
):
    new_velocities = velocities.copy()
    for i in range(len(positions)):
        if positions[i][2] <= 0:
            laplacian = np.zeros_like(velocities[i])
            weight_sum = 0.0
            for j in neighbours[i]:
                w = cotangent_weights[i][j] if j in cotangent_weights[i] else 1.0
                laplacian += w * (velocities[j] - velocities[i])
                weight_sum += w
            if weight_sum > 0:
                laplacian /= weight_sum
            new_velocities[i] = (1.0 - mu * dt) * velocities[i] + nabla * dt * laplacian
    return positions, new_velocities


def apply_surface_tension(
    positions, velocities, cotangent_weights, neighbours, faces, dt, gamma
):

    if gamma == 0:
        return velocities, positions

    n, dim = positions.shape
    mass_matrix = M(n, rho=1.0, positions=positions, faces=faces)
    vertex_masses = np.diag(mass_matrix)
    vertex_masses = np.where(vertex_masses > 1e-12, vertex_masses, 1.0)

    curvature = (np.linalg.inv(mass_matrix) @ L(n, cotangent_weights, neighbours)) @ positions
    normals = compute_vertex_normals(positions, faces)

    curvature_normal = np.sum(curvature * normals, axis=1)
    weighted_mean = np.sum(vertex_masses * curvature_normal) / np.sum(vertex_masses)

    # Remove the average normal motion so the flow preserves volume instead of shrinking the droplet.
    normal_motion = gamma * (curvature_normal - weighted_mean)
    positions_new = positions + dt * normal_motion[:, None] * normals
    velocities_new = (positions_new - positions) / dt
    return velocities_new, positions_new


def compute_closed_volume(positions, faces):
    """
    Compute the signed volume enclosed by a closed triangle mesh.

    The mesh is treated as a set of tetrahedra formed by each face and the origin.
    For a consistently oriented closed mesh, the sum of signed tetra volumes gives
    the enclosed volume up to sign.

    Args:
        positions (np.ndarray): (n, 3) array of vertex positions.
        faces (np.ndarray): (f, 3) array of triangle indices.

    Returns:
        float: Absolute enclosed volume.
    """

    volume = 0.0
    for i0, i1, i2 in faces:
        a = positions[i0]
        b = positions[i1]
        c = positions[i2]
        volume += np.dot(a, np.cross(b, c)) / 6.0
    return abs(volume)


def compute_boundary_mask(faces, n_vertices):
    """
    Detect boundary vertices by counting mesh edges that appear in only one face.

    Args:
        faces (np.ndarray): (f, 3) triangle index array.
        n_vertices (int): Number of mesh vertices.

    Returns:
        np.ndarray: Boolean mask of shape (n_vertices,), True for boundary vertices.
    """

    edge_counts = {}
    for i0, i1, i2 in faces:
        for a, b in ((i0, i1), (i1, i2), (i2, i0)):
            edge = tuple(sorted((int(a), int(b))))
            edge_counts[edge] = edge_counts.get(edge, 0) + 1

    boundary_mask = np.zeros(n_vertices, dtype=bool)
    for (i, j), count in edge_counts.items():
        if count == 1:
            boundary_mask[i] = True
            boundary_mask[j] = True
    return boundary_mask


def compute_vertex_normals(positions, faces):
    """
    Compute area-weighted vertex normals for a closed triangle mesh.

    Args:
        positions (np.ndarray): (n, 3) vertex positions.
        faces (np.ndarray): (f, 3) triangle indices.

    Returns:
        np.ndarray: (n, 3) array of unit normals.
    """

    normals = np.zeros_like(positions, dtype=np.float32)
    centroid = positions.mean(axis=0)

    for i0, i1, i2 in faces:
        a = positions[i0]
        b = positions[i1]
        c = positions[i2]
        face_normal = np.cross(b - a, c - a)
        face_center = (a + b + c) / 3.0
        if np.dot(face_normal, face_center - centroid) < 0:
            face_normal = -face_normal

        normals[i0] += face_normal
        normals[i1] += face_normal
        normals[i2] += face_normal

    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.where(norms > 1e-12, norms, 1.0)
    return normals / norms


def apply_volume_correction(
    positions, faces, target_volume, stiffness=0.15, tolerance=0.05
):
    """
    Push the surface along outward normals to correct drift in enclosed volume.

    This keeps the droplet rounder than a pure rescale because the update acts
    locally on the surface normal instead of shrinking or expanding the mesh
    isotropically about its centroid.

    Args:
        positions (np.ndarray): (n, 3) array of vertex positions.
        faces (np.ndarray): (f, 3) array of triangle indices.
        target_volume (float): Desired enclosed volume.
        stiffness (float): Blend factor in [0, 1] controlling correction strength.

    Returns:
        np.ndarray: Corrected positions array.
    """

    if target_volume <= 1e-12:
        return positions

    current_volume = compute_closed_volume(positions, faces)
    if current_volume <= 1e-12:
        return positions

    relative_error = abs(current_volume - target_volume) / target_volume
    if relative_error < tolerance:
        return positions

    surface_area = 0.0
    for i0, i1, i2 in faces:
        a = positions[i0]
        b = positions[i1]
        c = positions[i2]
        surface_area += 0.5 * np.linalg.norm(np.cross(b - a, c - a))

    if surface_area <= 1e-12:
        return positions

    normal_step = stiffness * (target_volume - current_volume) / surface_area
    normal_step = np.clip(normal_step, -0.03, 0.03)

    normals = compute_vertex_normals(positions, faces)
    return positions + normal_step * normals


def step(
    positions,
    velocities,
    neighbours,
    faces,
    cotangent_weights,
    dt=0.01,
    g=np.array([0, 0, -9.81]),
    friction_coeff=0.2,
    mu=0.3,
    nabla=0.05,
    gamma=0.1,
    target_volume=None,
    volume_stiffness=0.5,
):
    new_velocities = copy.deepcopy(velocities)

    # Gravity updates velocity first.
    new_positions, new_velocities = apply_gravity(
        positions.copy(), new_velocities, g, dt
    )

    # Advect the mesh using the updated velocity so gravity actually moves it.
    new_positions = new_positions + new_velocities * dt

    # Collision and friction
    new_positions, new_velocities = apply_collision_and_friction(
        new_positions, new_velocities, friction_coeff
    )

    # Viscosity
    new_positions, new_velocities = apply_viscosity(
        new_positions, new_velocities, neighbours, cotangent_weights, dt, mu, nabla
    )

    # Surface tension
    new_velocities, new_positions = apply_surface_tension(
        new_positions, new_velocities, cotangent_weights, neighbours, faces, dt, gamma
    )

    # Volume preservation
    if target_volume is not None:
        corrected_positions = apply_volume_correction(
            new_positions,
            faces,
            target_volume,
            stiffness=volume_stiffness,
        )
        new_velocities = (corrected_positions - new_positions) / dt + new_velocities
        new_positions = corrected_positions

    return new_positions, new_velocities
