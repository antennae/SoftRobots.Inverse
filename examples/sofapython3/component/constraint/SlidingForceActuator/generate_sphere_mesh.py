"""
Generate the sphere meshes used by the sliding force actuator examples.

Writes mesh/sphere.vtk (tetrahedra of the FEM model) and mesh/sphere.stl (finer
surface the actuators slide on, also used for the visual model). Run
build_spherical_param.py afterwards to regenerate mesh/sphere.spar.

Usage:
    python generate_sphere_mesh.py

Requires gmsh and meshio.
"""

from pathlib import Path

import gmsh
import meshio

RADIUS = 30.0  # mm
VOLUME_MESH_SIZE = 6.0  # mm, tetrahedra of the FEM model
SURFACE_MESH_SIZE = 2.5  # mm, triangles of the surface the actuators slide on

mesh_dir = Path(__file__).resolve().parent / 'mesh'
mesh_dir.mkdir(exist_ok=True)


def generate(mesh_size, dim):
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    gmsh.model.add('sphere')
    gmsh.model.occ.addSphere(0, 0, 0, RADIUS)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', mesh_size)
    gmsh.option.setNumber('Mesh.MeshSizeMax', mesh_size)
    gmsh.model.mesh.generate(dim)
    msh_path = mesh_dir / 'sphere.msh'
    gmsh.write(str(msh_path))
    gmsh.finalize()
    mesh = meshio.read(str(msh_path))
    msh_path.unlink()
    return mesh


volume = generate(VOLUME_MESH_SIZE, 3)
meshio.write(
    str(mesh_dir / 'sphere.vtk'),
    meshio.Mesh(volume.points, [('tetra', volume.cells_dict['tetra'])]),
    file_format='vtk42',
    binary=False,
)

surface = generate(SURFACE_MESH_SIZE, 2)
meshio.write(
    str(mesh_dir / 'sphere.stl'),
    meshio.Mesh(surface.points, [('triangle', surface.cells_dict['triangle'])]),
    binary=True,
)
print(
    f"sphere.vtk: {len(volume.points)} points, {len(volume.cells_dict['tetra'])} tetrahedra\n"
    f"sphere.stl: {len(surface.cells_dict['triangle'])} triangles"
)
