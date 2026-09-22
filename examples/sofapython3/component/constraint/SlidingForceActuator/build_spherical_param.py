"""
Build the spherical parameterization S^Par of a closed genus-0 surface mesh.

Produces the .spar file required by SphericalSlidingForceActuator and
AreaContactSlidingForceActuator (Data field "sparFile").

Implements the Mocanu & Zaharia (2011) algorithm:
  1. Compute approximate Gaussian curvature K(v) per vertex  (Eq. 2)
  2. Iteratively flatten high-curvature vertices              (Eq. 3)
  3. Project onto unit sphere                                  (Eq. 4)
  4. Area-based smoothing to equalize triangle areas           (Eq. 5)

Reference: CalculusNote_feb2022.pdf — Tanguy's cable placement
           optimization for a heart phantom.

Output: <mesh stem>.npz and <mesh stem>.spar (binary, read by the C++ components)
  - vertices_spar   (V, 3) — vertex positions on unit sphere
  - theta_phi       (V, 2) — per-vertex (θ, φ)
  - triangles       (F, 3) — triangle connectivity (same as original)
  - vertices_original (V, 3) — original mesh vertices

Usage:
    python build_spherical_param.py --mesh mesh/sphere.stl
    python build_spherical_param.py --mesh mesh/sphere.stl --pole 0,0,-1
    python build_spherical_param.py --mesh mesh/sphere.stl --validate --plot

Requires numpy and trimesh (matplotlib only for --plot).
"""

import argparse
import struct
from pathlib import Path

import numpy as np
import trimesh

# ── Mesh utilities ──────────────────────────────────────────────


def extract_largest_component(mesh):
    """Extract the largest connected component from a trimesh mesh.

    Returns a new Trimesh with re-indexed vertices/faces.
    """
    cc = trimesh.graph.connected_components(mesh.face_adjacency)
    largest = max(cc, key=len)
    sub = mesh.submesh([largest], append=True)
    return sub


# ── Step 1: Gaussian curvature (Eq. 2) ─────────────────────────


def _vertex_angles(vertices, faces):
    """Compute the angle at each vertex in each triangle.

    Returns (F, 3) array where entry [f, i] is the angle at
    vertex faces[f, i] in triangle f.
    """
    v0 = vertices[faces[:, 0]]
    v1 = vertices[faces[:, 1]]
    v2 = vertices[faces[:, 2]]

    # edges emanating from each vertex of the triangle
    # vertex 0: edges (v1-v0), (v2-v0)
    # vertex 1: edges (v0-v1), (v2-v1)
    # vertex 2: edges (v0-v2), (v1-v2)
    edges = [
        (v1 - v0, v2 - v0),
        (v0 - v1, v2 - v1),
        (v0 - v2, v1 - v2),
    ]

    angles = np.empty((len(faces), 3), dtype=np.float64)
    for i, (e1, e2) in enumerate(edges):
        cos_a = np.einsum('ij,ij->i', e1, e2) / (
            np.linalg.norm(e1, axis=1) * np.linalg.norm(e2, axis=1) + 1e-30
        )
        cos_a = np.clip(cos_a, -1.0, 1.0)
        angles[:, i] = np.arccos(cos_a)

    return angles


def compute_gaussian_curvature(vertices, faces):
    """Approximate Gaussian curvature K(v) per vertex (Eq. 2).

    K(v) = (2π - Σ α_t(v)) / (ξ + Σ A_t/3)

    where α_t(v) is the angle at v in triangle t,
    A_t is the area of triangle t, and ξ is the mean triangle area.
    """
    n_verts = len(vertices)
    angles = _vertex_angles(vertices, faces)

    # face areas
    v0 = vertices[faces[:, 0]]
    v1 = vertices[faces[:, 1]]
    v2 = vertices[faces[:, 2]]
    face_areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    xi = face_areas.mean()

    # accumulate angle deficit and area per vertex
    angle_sum = np.zeros(n_verts, dtype=np.float64)
    area_sum = np.zeros(n_verts, dtype=np.float64)

    for i in range(3):
        np.add.at(angle_sum, faces[:, i], angles[:, i])
        np.add.at(area_sum, faces[:, i], face_areas / 3.0)

    K = (2.0 * np.pi - angle_sum) / (xi + area_sum)
    return K


# ── Step 2: Curvature-driven flattening (Eq. 3) ────────────────


def _build_adjacency(n_verts, faces):
    """Build vertex adjacency list from faces."""
    adj: list[list[int]] = [[] for _ in range(n_verts)]
    for f in faces:
        for i in range(3):
            vi = f[i]
            for j in range(3):
                if i != j:
                    vj = f[j]
                    if vj not in adj[vi]:
                        adj[vi].append(vj)
    # convert to numpy arrays for speed
    return [np.array(a, dtype=np.int64) for a in adj]


def _check_all_visible(vertices, center, faces):
    """Check if all vertices are visible from center (ray test).

    A vertex is visible if the ray from center to vertex doesn't
    intersect any triangle (other than those containing the vertex).
    We use a simplified approach: check if the mesh is star-shaped
    w.r.t. center by verifying that for each vertex, the ray
    center→vertex doesn't cross any face.
    """
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    directions = vertices - center
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-30)
    directions = directions / norms

    # build set of faces per vertex for exclusion
    vert_faces: list[set[int]] = [set() for _ in range(len(vertices))]
    for fi, f in enumerate(faces):
        for vi in f:
            vert_faces[vi].add(fi)

    # use trimesh ray casting
    origins = np.tile(center, (len(vertices), 1))
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        origins, directions, multiple_hits=True
    )

    invisible = set()
    for hit_idx in range(len(index_ray)):
        ray_i = index_ray[hit_idx]
        tri_i = index_tri[hit_idx]
        # skip faces that contain this vertex
        if tri_i in vert_faces[ray_i]:
            continue
        # check if intersection is between center and vertex
        hit_dist = np.linalg.norm(locations[hit_idx] - center)
        vert_dist = float(norms[ray_i])
        if hit_dist < vert_dist - 1e-6:
            invisible.add(ray_i)

    return invisible


def flatten_mesh(
    vertices, faces, max_iterations=5000, batch_fraction=0.05, verbose=True
):
    """Iteratively flatten the mesh by moving high-curvature
    vertices to their neighbor centroids (Eq. 3).

    Continues until all vertices are visible from the gravity center,
    or max_iterations is reached.

    Uses batch processing: each iteration moves the top
    `batch_fraction` of high-curvature vertices simultaneously.
    """
    verts = vertices.copy()
    center = verts.mean(axis=0)
    adj = _build_adjacency(len(verts), faces)

    for it in range(max_iterations):
        K = compute_gaussian_curvature(verts, faces)
        K_abs = np.abs(K)

        # Move top batch_fraction vertices with highest |K|
        n_batch = max(1, int(len(verts) * batch_fraction))
        top_indices = np.argsort(K_abs)[-n_batch:]

        max_displacement = 0.0
        for vi in top_indices:
            neighbors = adj[vi]
            if len(neighbors) == 0:
                continue
            new_pos = verts[neighbors].mean(axis=0)
            max_displacement = max(
                max_displacement, np.linalg.norm(new_pos - verts[vi])
            )
            verts[vi] = new_pos

        # recompute center
        center = verts.mean(axis=0)

        if verbose and (it + 1) % 100 == 0:
            print(
                f'  Flatten iter {it+1}: max|K|={K_abs.max():.4f}, '
                f'max_disp={max_displacement:.6f}'
            )

        # convergence: displacements are tiny
        if max_displacement < 1e-6:
            if verbose:
                print(
                    f'  Converged at iteration {it+1} '
                    f'(max_displacement={max_displacement:.2e})'
                )
            break

    # Check visibility
    invisible = _check_all_visible(verts, center, faces)
    if verbose:
        print(
            f'  After flattening: {len(invisible)} vertices '
            f'not visible from center '
            f'(out of {len(verts)})'
        )

    return verts, center


# ── Step 3: Sphere projection (Eq. 4) ──────────────────────────


def project_to_sphere(vertices, center, pole_direction=None):
    """Project vertices onto unit sphere centered at `center`.

    v' = (v - center) / ||v - center||   (Eq. 4)

    If pole_direction is given (3-vector in original mesh coords),
    the sphere is rotated so that direction maps to θ=0 (+Z pole).
    This makes the parameterization independent of mesh orientation.
    """
    centered = vertices - center
    norms = np.linalg.norm(centered, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-30)
    sph = centered / norms

    if pole_direction is not None:
        # Rotate sphere so pole_direction aligns with +Z
        d = np.asarray(pole_direction, dtype=np.float64)
        d = d / np.linalg.norm(d)
        z = np.array([0.0, 0.0, 1.0])

        if np.allclose(d, z):
            return sph
        if np.allclose(d, -z):
            # 180-degree rotation around X
            sph[:, 2] = -sph[:, 2]
            sph[:, 1] = -sph[:, 1]
            return sph

        # Rodrigues rotation: rotate d to z
        axis = np.cross(d, z)
        axis = axis / np.linalg.norm(axis)
        cos_a = np.dot(d, z)
        sin_a = np.sqrt(1 - cos_a**2)
        K = np.array(
            [
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0],
            ]
        )
        R = np.eye(3) + sin_a * K + (1 - cos_a) * (K @ K)
        sph = (R @ sph.T).T

    return sph


# ── Step 4: Area-based smoothing (Eq. 5) ────────────────────────


def _triangle_area_on_sphere(v0, v1, v2):
    """Area of a single triangle given 3D vertex positions."""
    return 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))


def _triangle_areas_batch(vertices, faces):
    """Compute all triangle areas. Returns (F,) array."""
    v0 = vertices[faces[:, 0]]
    v1 = vertices[faces[:, 1]]
    v2 = vertices[faces[:, 2]]
    return 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)


def _check_triangle_inversion(
    vertices, faces, vert_faces_list, vi, new_pos, old_pos
):
    """Check if moving vertex vi to new_pos inverts any adjacent
    triangle. Returns True if any triangle would invert."""
    for fi in vert_faces_list[vi]:
        f = faces[fi]
        v = [
            vertices[f[0]].copy(),
            vertices[f[1]].copy(),
            vertices[f[2]].copy(),
        ]
        # find which local index is vi
        local_i = np.where(f == vi)[0][0]
        # compute normal with old position
        old_normal = np.cross(
            v[(local_i + 1) % 3] - v[local_i],
            v[(local_i + 2) % 3] - v[local_i],
        )
        # compute normal with new position
        v[local_i] = new_pos
        new_normal = np.cross(
            v[(local_i + 1) % 3] - v[local_i],
            v[(local_i + 2) % 3] - v[local_i],
        )
        # inverted if normals point in opposite directions
        if np.dot(old_normal, new_normal) < 0:
            return True
    return False


def _spherical_to_cart(theta, phi):
    """Convert spherical (θ, φ) to unit sphere Cartesian."""
    return np.array(
        [
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta),
        ]
    )


def _cart_to_spherical(xyz):
    """Convert Cartesian to spherical (θ, φ).
    θ ∈ (0, π), φ ∈ [0, 2π)."""
    r = np.linalg.norm(xyz)
    theta = np.arccos(np.clip(xyz[2] / max(r, 1e-30), -1, 1))
    phi = np.arctan2(xyz[1], xyz[0]) % (2 * np.pi)
    return theta, phi


def area_smoothing(
    vertices_spar, faces, max_iterations=200, lr=0.3, tol=1e-8, verbose=True
):
    """Area-based smoothing on the unit sphere (Eq. 5).

    Laplacian smoothing: moves each vertex toward the centroid of
    its neighbors on the sphere, then re-projects to the unit sphere.
    This keeps area ratios well-bounded while improving uniformity.
    Topology is preserved by rejecting moves that invert triangles.

    Parameters
    ----------
    lr : float
        Blend factor toward neighbor centroid (0=no move, 1=full).
    """
    verts = vertices_spar.copy()
    n_verts = len(verts)
    n_faces = len(faces)

    # Build per-vertex face list and adjacency
    vert_faces: list[list[int]] = [[] for _ in range(n_verts)]
    for fi in range(n_faces):
        for vi in faces[fi]:
            vert_faces[vi].append(fi)

    adj = _build_adjacency(n_verts, faces)

    for it in range(max_iterations):
        areas = _triangle_areas_batch(verts, faces)
        area_cv = areas.std() / (areas.mean() + 1e-30)

        if area_cv < tol:
            if verbose:
                print(
                    f'  Converged at iteration {it+1} '
                    f'(area_CV={area_cv:.2e})'
                )
            break

        new_verts = verts.copy()
        n_moved = 0

        for vi in range(n_verts):
            neighbors = adj[vi]
            if len(neighbors) == 0:
                continue

            centroid = verts[neighbors].mean(axis=0)
            candidate = (1 - lr) * verts[vi] + lr * centroid
            norm = np.linalg.norm(candidate)
            if norm < 1e-30:
                continue
            candidate = candidate / norm

            if _check_triangle_inversion(
                verts, faces, vert_faces, vi, candidate, verts[vi]
            ):
                continue

            new_verts[vi] = candidate
            n_moved += 1

        verts = new_verts

        if verbose and (it + 1) % 10 == 0:
            areas_new = _triangle_areas_batch(verts, faces)
            area_cv_new = areas_new.std() / (areas_new.mean() + 1e-30)
            print(
                f'  Smooth iter {it+1}: area_CV={area_cv_new:.4f}, '
                f'moved={n_moved}/{n_verts}'
            )

    return verts


# ── Barycentric / mapping utilities (Eq. 6, 7) ─────────────────


def compute_barycentric(v0, v1, v2, p):
    """Compute barycentric coords (α, β) of point p in triangle
    (v0, v1, v2) using Eq. 7.

    Returns (alpha, beta) such that p ≈ v0 + α(v1-v0) + β(v2-v0).
    """
    M = np.cross(v1 - v0, v2 - v0)
    M_sq = np.dot(M, M)
    if M_sq < 1e-30:
        return 0.0, 0.0
    alpha = np.dot(M, np.cross(p - v0, v2 - v0)) / M_sq
    beta = np.dot(M, np.cross(v1 - v0, p - v0)) / M_sq
    return float(alpha), float(beta)


def spherical_to_mesh(theta, phi, verts_spar, verts_orig, faces):
    """Map spherical coords (θ,φ) to mesh position.

    1. Convert (θ,φ) to point on unit sphere
    2. Find containing triangle on S^Par
    3. Compute barycentric coords (α, β)
    4. Map to original mesh via same barycentric coords (Eq. 6)
    """
    p_sph = _spherical_to_cart(theta, phi)

    # find containing triangle by checking all triangles
    tri_idx, alpha, beta = _find_triangle_on_sphere(p_sph, verts_spar, faces)

    if tri_idx < 0:
        # fallback: closest triangle via trimesh
        mesh_spar = trimesh.Trimesh(
            vertices=verts_spar, faces=faces, process=False
        )
        _, _, tri_idx = trimesh.proximity.closest_point(mesh_spar, [p_sph])
        tri_idx = int(tri_idx[0])
        f = faces[tri_idx]
        alpha, beta, _ = _radial_barycentric(
            verts_spar[f[0]], verts_spar[f[1]], verts_spar[f[2]], p_sph
        )

    # map to original mesh (Eq. 6)
    f = faces[tri_idx]
    pos = (
        verts_orig[f[0]]
        + alpha * (verts_orig[f[1]] - verts_orig[f[0]])
        + beta * (verts_orig[f[2]] - verts_orig[f[0]])
    )
    return pos, tri_idx, alpha, beta


def _radial_barycentric(v0, v1, v2, p_sph):
    """Compute barycentric coords by projecting p_sph radially
    (from origin) onto the triangle plane, then computing standard
    barycentric coords of the intersection point.

    This is the correct containment test for spherical
    parameterization: we want the ray from origin through p_sph
    to hit the triangle.
    """
    M = np.cross(v1 - v0, v2 - v0)
    M_dot_p = np.dot(M, p_sph)
    if abs(M_dot_p) < 1e-30:
        return 0.0, 0.0, False  # ray parallel to triangle
    t = np.dot(M, v0) / M_dot_p
    if t < 0:
        return 0.0, 0.0, False  # triangle behind origin
    # intersection point on triangle plane
    p_plane = t * p_sph
    M_sq = np.dot(M, M)
    alpha = np.dot(M, np.cross(p_plane - v0, v2 - v0)) / M_sq
    beta = np.dot(M, np.cross(v1 - v0, p_plane - v0)) / M_sq
    return float(alpha), float(beta), True


def _find_triangle_on_sphere(p_sph, verts_spar, faces):
    """Find the triangle on S^Par containing point p_sph.

    Uses radial projection from origin: shoots a ray from (0,0,0)
    through p_sph and finds the triangle it intersects.
    """
    best_tri = -1
    best_alpha = 0.0
    best_beta = 0.0
    best_dist = 1e30

    for fi in range(len(faces)):
        f = faces[fi]
        v0 = verts_spar[f[0]]
        v1 = verts_spar[f[1]]
        v2 = verts_spar[f[2]]

        alpha, beta, valid = _radial_barycentric(v0, v1, v2, p_sph)
        if not valid:
            continue

        gamma = 1.0 - alpha - beta

        if alpha >= -1e-8 and beta >= -1e-8 and gamma >= -1e-8:
            return fi, alpha, beta

        # track closest for fallback
        alpha_c = np.clip(alpha, 0, 1)
        beta_c = np.clip(beta, 0, min(1, 1 - alpha_c))
        proj = v0 + alpha_c * (v1 - v0) + beta_c * (v2 - v0)
        proj_sph = proj / max(np.linalg.norm(proj), 1e-30)
        dist = np.linalg.norm(proj_sph - p_sph)
        if dist < best_dist:
            best_dist = dist
            best_tri = fi
            best_alpha = alpha_c
            best_beta = beta_c

    return best_tri, best_alpha, best_beta


# ── Jacobian computation (Eq. 21) ──────────────────────────────


def compute_sliding_jacobian(theta, phi, tri_idx, verts_spar, faces):
    """Compute ∂α/∂θ, ∂β/∂θ, ∂α/∂φ, ∂β/∂φ for the constraint
    Jacobian sliding rows.

    From Eq. 7 differentiated w.r.t. θ and φ:
      ∂P_sph/∂θ = (cosθ cosφ, cosθ sinφ, -sinθ)
      ∂P_sph/∂φ = (-sinθ sinφ, sinθ cosφ, 0)
    """
    f = faces[tri_idx]
    v0 = verts_spar[f[0]]
    v1 = verts_spar[f[1]]
    v2 = verts_spar[f[2]]

    M = np.cross(v1 - v0, v2 - v0)
    M_sq = np.dot(M, M)
    if M_sq < 1e-30:
        return 0, 0, 0, 0

    M_over_Msq = M / M_sq

    # derivatives of P_sph
    dP_dtheta = np.array(
        [
            np.cos(theta) * np.cos(phi),
            np.cos(theta) * np.sin(phi),
            -np.sin(theta),
        ]
    )
    dP_dphi = np.array(
        [
            -np.sin(theta) * np.sin(phi),
            np.sin(theta) * np.cos(phi),
            0.0,
        ]
    )

    # ∂α/∂θ = (M/||M||²) · (∂P/∂θ × (v2 - v0))
    da_dtheta = np.dot(M_over_Msq, np.cross(dP_dtheta, v2 - v0))
    # ∂β/∂θ = (M/||M||²) · ((v1 - v0) × ∂P/∂θ)
    db_dtheta = np.dot(M_over_Msq, np.cross(v1 - v0, dP_dtheta))

    # ∂α/∂φ = (M/||M||²) · (∂P/∂φ × (v2 - v0))
    da_dphi = np.dot(M_over_Msq, np.cross(dP_dphi, v2 - v0))
    # ∂β/∂φ = (M/||M||²) · ((v1 - v0) × ∂P/∂φ)
    db_dphi = np.dot(M_over_Msq, np.cross(v1 - v0, dP_dphi))

    return da_dtheta, db_dtheta, da_dphi, db_dphi


# ── Binary .spar format for C++ consumption ────────────────────


def save_spar_binary(path, verts_spar, theta_phi, faces, verts_orig):
    """Save S^Par data in a simple binary format readable in C++.

    Format:
      magic: 4 bytes "SPAR"
      version: uint32
      n_vertices: uint32
      n_faces: uint32
      vertices_spar: n_vertices × 3 × float64
      theta_phi: n_vertices × 2 × float64
      faces: n_faces × 3 × uint32
      vertices_original: n_vertices × 3 × float64
    """
    with open(path, 'wb') as f:
        f.write(b'SPAR')
        f.write(struct.pack('<I', 1))  # version
        f.write(struct.pack('<I', len(verts_spar)))
        f.write(struct.pack('<I', len(faces)))
        f.write(verts_spar.astype(np.float64).tobytes())
        f.write(theta_phi.astype(np.float64).tobytes())
        f.write(faces.astype(np.uint32).tobytes())
        f.write(verts_orig.astype(np.float64).tobytes())


# ── Validation ──────────────────────────────────────────────────


def validate(verts_spar, theta_phi, faces, verts_orig, do_plot, plot_path=None):
    """Run validation checks on the spherical parameterization."""
    print('\n=== Validation ===')

    # 1. Bijectivity: all triangle normals should point outward
    print('\n1. Bijectivity (triangle normals point outward):')
    v0 = verts_spar[faces[:, 0]]
    v1 = verts_spar[faces[:, 1]]
    v2 = verts_spar[faces[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0)
    centroids = (v0 + v1 + v2) / 3.0
    dots = np.einsum('ij,ij->i', normals, centroids)
    n_inverted = np.sum(dots < 0)
    print(f'   Inverted triangles: {n_inverted} / {len(faces)}')
    if n_inverted > 0:
        print(f'   WARNING: {n_inverted} triangles are inverted!')

    # 2. Area distortion
    print('\n2. Area distortion:')
    areas_orig = 0.5 * np.linalg.norm(
        np.cross(
            verts_orig[faces[:, 1]] - verts_orig[faces[:, 0]],
            verts_orig[faces[:, 2]] - verts_orig[faces[:, 0]],
        ),
        axis=1,
    )
    areas_spar = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    # normalize both to sum to 1
    areas_orig_n = areas_orig / areas_orig.sum()
    areas_spar_n = areas_spar / areas_spar.sum()
    ratio = areas_spar_n / (areas_orig_n + 1e-30)
    print('   Area ratio (spar/orig, normalized):')
    print(
        f'     min={ratio.min():.4f}, max={ratio.max():.4f}, '
        f'mean={ratio.mean():.4f}, std={ratio.std():.4f}'
    )

    # 3. Round-trip test
    print('\n3. Round-trip test (random points):')
    n_test = 50
    rng = np.random.default_rng(42)
    errors: list[float] = []
    for _ in range(n_test):
        # pick random triangle and random barycentric
        fi = rng.integers(0, len(faces))
        f = faces[fi]
        u, v = rng.random(), rng.random()
        if u + v > 1:
            u, v = 1 - u, 1 - v
        # point on sphere
        p_sph = (
            verts_spar[f[0]]
            + u * (verts_spar[f[1]] - verts_spar[f[0]])
            + v * (verts_spar[f[2]] - verts_spar[f[0]])
        )
        # normalize to sphere
        p_sph = p_sph / np.linalg.norm(p_sph)
        theta, phi = _cart_to_spherical(p_sph)

        # map to mesh and back
        mesh_pos, tri_found, alpha, beta = spherical_to_mesh(
            theta, phi, verts_spar, verts_orig, faces
        )
        # map mesh_pos back to sphere via the found triangle
        f2 = faces[tri_found]
        p_sph_back = (
            verts_spar[f2[0]]
            + alpha * (verts_spar[f2[1]] - verts_spar[f2[0]])
            + beta * (verts_spar[f2[2]] - verts_spar[f2[0]])
        )
        p_sph_back = p_sph_back / np.linalg.norm(p_sph_back)
        theta_back, phi_back = _cart_to_spherical(p_sph_back)

        err = np.sqrt((theta - theta_back) ** 2 + (phi - phi_back) ** 2)
        errors.append(err)

    errors_arr = np.asarray(errors, dtype=np.float64)
    print(
        f'   Angular error (rad): mean={errors_arr.mean():.2e}, '
        f'max={errors_arr.max():.2e}'
    )

    # 4. Jacobian check (numeric vs analytic)
    print('\n4. Jacobian check (numeric vs analytic):')
    n_jac_test = 20
    jac_errors: list[float] = []
    eps = 1e-6
    for _ in range(n_jac_test):
        fi = rng.integers(0, len(faces))
        f = faces[fi]
        u, v = 0.3, 0.3  # safe interior point
        p_sph = (
            verts_spar[f[0]]
            + u * (verts_spar[f[1]] - verts_spar[f[0]])
            + v * (verts_spar[f[2]] - verts_spar[f[0]])
        )
        p_sph = p_sph / np.linalg.norm(p_sph)
        theta, phi = _cart_to_spherical(p_sph)

        # analytic
        da_dt, db_dt, da_dp, db_dp = compute_sliding_jacobian(
            theta, phi, fi, verts_spar, faces
        )

        # numeric (finite differences)
        def _bary(t, p):
            ps = _spherical_to_cart(t, p)
            return compute_barycentric(
                verts_spar[f[0]], verts_spar[f[1]], verts_spar[f[2]], ps
            )

        a0, b0 = _bary(theta, phi)
        a_dt, b_dt = _bary(theta + eps, phi)
        a_dp, b_dp = _bary(theta, phi + eps)

        da_dt_num = (a_dt - a0) / eps
        db_dt_num = (b_dt - b0) / eps
        da_dp_num = (a_dp - a0) / eps
        db_dp_num = (b_dp - b0) / eps

        err = np.sqrt(
            (da_dt - da_dt_num) ** 2
            + (db_dt - db_dt_num) ** 2
            + (da_dp - da_dp_num) ** 2
            + (db_dp - db_dp_num) ** 2
        )
        jac_errors.append(err)

    jac_errors_arr = np.asarray(jac_errors, dtype=np.float64)
    print(
        f'   Jacobian error: mean={jac_errors_arr.mean():.2e}, '
        f'max={jac_errors_arr.max():.2e}'
    )

    # 5. Plots
    if do_plot:
        _plot_validation(
            verts_spar,
            theta_phi,
            faces,
            verts_orig,
            areas_orig_n,
            areas_spar_n,
            ratio,
            plot_path,
        )


def _plot_validation(
    verts_spar,
    theta_phi,
    faces,
    verts_orig,
    areas_orig_n,
    areas_spar_n,
    ratio,
    out_path=None,
):
    """Generate validation plots."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(18, 12))

    # 1. Original mesh
    ax1 = fig.add_subplot(231, projection='3d')
    ax1.plot_trisurf(
        verts_orig[:, 0],
        verts_orig[:, 1],
        verts_orig[:, 2],
        triangles=faces,
        color='lightblue',
        edgecolor='gray',
        linewidth=0.1,
        alpha=0.7,
    )
    ax1.set_title('Original mesh')

    # 2. S^Par on unit sphere
    ax2 = fig.add_subplot(232, projection='3d')
    ax2.plot_trisurf(
        verts_spar[:, 0],
        verts_spar[:, 1],
        verts_spar[:, 2],
        triangles=faces,
        color='lightyellow',
        edgecolor='gray',
        linewidth=0.1,
        alpha=0.7,
    )
    ax2.set_title('S^Par (unit sphere)')

    # 3. θ-φ plot
    ax3 = fig.add_subplot(233)
    ax3.triplot(
        theta_phi[:, 1],
        theta_phi[:, 0],
        faces,
        linewidth=0.3,
        color='gray',
    )
    ax3.set_xlabel('φ')
    ax3.set_ylabel('θ')
    ax3.set_title('(θ, φ) parameterization')
    ax3.invert_yaxis()

    # 4. Area distortion histogram
    ax4 = fig.add_subplot(234)
    ax4.hist(ratio, bins=50, color='steelblue', edgecolor='black')
    ax4.axvline(1.0, color='red', linestyle='--', label='ideal')
    ax4.set_xlabel('Area ratio (S^Par / original, normalized)')
    ax4.set_ylabel('Count')
    ax4.set_title('Area distortion')
    ax4.legend()

    # 5. Area on sphere colored by distortion
    ax5 = fig.add_subplot(235, projection='3d')
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    norm = Normalize(vmin=ratio.min(), vmax=min(ratio.max(), 5.0))
    sm = ScalarMappable(cmap='RdYlGn_r', norm=norm)
    sm.set_array(ratio)
    colors = sm.to_rgba(ratio)
    tri_verts = verts_spar[faces]  # (F, 3, 3)
    poly = Poly3DCollection(
        tri_verts, linewidth=0.05, edgecolor='gray', alpha=0.9
    )
    poly.set_facecolor(colors)
    ax5.add_collection3d(poly)
    ax5.auto_scale_xyz(verts_spar[:, 0], verts_spar[:, 1], verts_spar[:, 2])
    ax5.set_title('S^Par colored by area distortion')
    fig.colorbar(sm, ax=ax5, shrink=0.6, label='area ratio')

    # 6. Per-vertex (θ,φ) as scatter colored by curvature
    ax6 = fig.add_subplot(236)
    K_orig = compute_gaussian_curvature(verts_orig, faces)
    sc = ax6.scatter(
        theta_phi[:, 1],
        theta_phi[:, 0],
        c=K_orig,
        cmap='coolwarm',
        s=2,
        vmin=-np.percentile(np.abs(K_orig), 95),
        vmax=np.percentile(np.abs(K_orig), 95),
    )
    ax6.set_xlabel('φ')
    ax6.set_ylabel('θ')
    ax6.set_title('(θ,φ) colored by original K(v)')
    ax6.invert_yaxis()
    fig.colorbar(sc, ax=ax6, shrink=0.6, label='K(v)')

    plt.tight_layout()
    if out_path is None:
        out_path = Path('spar_validation.png')
    fig.savefig(str(out_path), dpi=200, bbox_inches='tight')
    print(f'\nValidation plot saved to {out_path}')
    plt.close(fig)


# ── Main ────────────────────────────────────────────────────────


def build_spherical_param(
    mesh_path,
    output_dir=None,
    flatten_iters=5000,
    smooth_iters=200,
    smooth_lr=0.01,
    pole_direction=None,
    verbose=True,
):
    """Build spherical parameterization from a closed genus-0 STL.

    Parameters
    ----------
    pole_direction : (3,) array or None
        Direction in original mesh coordinates that should map to
        θ=0 (+Z pole on the sphere). Default: centroid → max-Z vertex.
        Point it at a region the actuator never slides over (e.g. the
        fixed base), since (θ, φ) is singular at the poles.

    Returns (verts_spar, theta_phi, faces, verts_orig).
    """
    mesh_path = Path(mesh_path)
    if output_dir is None:
        output_dir = mesh_path.parent

    print(f'Loading mesh: {mesh_path}')
    mesh = trimesh.load(str(mesh_path))
    print(f'  Raw: {len(mesh.vertices)} verts, {len(mesh.faces)} ' f'faces')

    # Extract largest connected component (genus-0 surface)
    mesh = extract_largest_component(mesh)
    verts_orig = np.array(mesh.vertices, dtype=np.float64)
    faces = np.array(mesh.faces, dtype=np.int64)
    print(f'  Main component: {len(verts_orig)} verts, ' f'{len(faces)} faces')
    print(f'  Euler: {len(verts_orig) - 3*len(faces)//2 + len(faces)}')

    # Step 1-2: Curvature-driven flattening
    print('\nStep 1-2: Curvature-driven flattening...')
    verts_flat, center = flatten_mesh(
        verts_orig,
        faces,
        max_iterations=flatten_iters,
        verbose=verbose,
    )

    # Step 3: Project onto unit sphere
    # Default pole_direction: from centroid toward max-Z vertex
    if pole_direction is None:
        z_max_idx = np.argmax(verts_orig[:, 2])
        pole_direction = verts_orig[z_max_idx] - verts_orig.mean(axis=0)

    print(
        f'\nStep 3: Projecting onto unit sphere '
        f'(pole toward {pole_direction / np.linalg.norm(pole_direction)})...'
    )
    verts_spar = project_to_sphere(
        verts_flat, center, pole_direction=pole_direction
    )
    print(
        f'  Sphere radius check: '
        f'min={np.linalg.norm(verts_spar, axis=1).min():.6f}, '
        f'max={np.linalg.norm(verts_spar, axis=1).max():.6f}'
    )

    # Check for inverted triangles before smoothing
    v0 = verts_spar[faces[:, 0]]
    v1 = verts_spar[faces[:, 1]]
    v2 = verts_spar[faces[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0)
    centroids = (v0 + v1 + v2) / 3.0
    dots = np.einsum('ij,ij->i', normals, centroids)
    n_inv = np.sum(dots < 0)
    print(f'  Inverted triangles after projection: {n_inv}')

    # Flip inverted triangles by swapping vertex order
    if n_inv > 0:
        inv_mask = dots < 0
        faces[inv_mask] = faces[inv_mask][:, [0, 2, 1]]
        print(f'  Flipped {n_inv} triangles to fix normals')

    # Step 4: Area-based smoothing
    print(
        f'\nStep 4: Area-based smoothing '
        f'(max {smooth_iters} iterations)...'
    )
    verts_spar = area_smoothing(
        verts_spar,
        faces,
        max_iterations=smooth_iters,
        lr=smooth_lr,
        verbose=verbose,
    )

    # Compute (θ, φ) for each vertex
    theta_phi = np.zeros((len(verts_spar), 2), dtype=np.float64)
    for i in range(len(verts_spar)):
        theta_phi[i, 0], theta_phi[i, 1] = _cart_to_spherical(verts_spar[i])

    # Save outputs
    pole_dir_normalized = np.asarray(
        pole_direction, dtype=np.float64
    ) / np.linalg.norm(pole_direction)
    npz_path = Path(output_dir) / f'{mesh_path.stem}.npz'
    np.savez(
        str(npz_path),
        vertices_spar=verts_spar,
        theta_phi=theta_phi,
        triangles=faces,
        vertices_original=verts_orig,
        pole_direction=pole_dir_normalized,
    )
    print(f'\nSaved: {npz_path}')

    spar_path = Path(output_dir) / f'{mesh_path.stem}.spar'
    save_spar_binary(spar_path, verts_spar, theta_phi, faces, verts_orig)
    print(f'Saved: {spar_path}')

    return verts_spar, theta_phi, faces, verts_orig


def main():
    parser = argparse.ArgumentParser(
        description='Build spherical parameterization S^Par '
        'for a closed genus-0 surface mesh'
    )
    parser.add_argument(
        '--mesh',
        type=Path,
        required=True,
        help='Path to input surface STL (closed, genus-0)',
    )
    parser.add_argument(
        '--output_dir',
        type=Path,
        default=None,
        help='Output directory (default: same as mesh)',
    )
    parser.add_argument(
        '--flatten_iters',
        type=int,
        default=5000,
        help='Max flattening iterations',
    )
    parser.add_argument(
        '--smooth_iters',
        type=int,
        default=500,
        help='Max area-smoothing iterations',
    )
    parser.add_argument(
        '--smooth_lr',
        type=float,
        default=0.3,
        help='Learning rate for area smoothing',
    )
    parser.add_argument(
        '--validate',
        action='store_true',
        help='Run validation checks after building',
    )
    parser.add_argument(
        '--plot',
        action='store_true',
        help='Generate validation plots (requires --validate)',
    )
    parser.add_argument(
        '--pole',
        type=str,
        default=None,
        help='Pole direction in mesh coords as "x,y,z". '
        'This direction maps to θ=0 (+Z pole). '
        'Default: auto (centroid → max-Z vertex).',
    )
    args = parser.parse_args()

    pole_dir = None
    if args.pole is not None:
        pole_dir = np.array([float(x) for x in args.pole.split(',')])

    verts_spar, theta_phi, faces, verts_orig = build_spherical_param(
        args.mesh,
        output_dir=args.output_dir,
        flatten_iters=args.flatten_iters,
        smooth_iters=args.smooth_iters,
        smooth_lr=args.smooth_lr,
        pole_direction=pole_dir,
    )

    if args.validate:
        out_dir = args.output_dir or args.mesh.parent
        validate(
            verts_spar,
            theta_phi,
            faces,
            verts_orig,
            do_plot=args.plot,
            plot_path=Path(out_dir) / f'{args.mesh.stem}_spar_validation.png',
        )


if __name__ == '__main__':
    main()
