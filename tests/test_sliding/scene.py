"""SOFA Scene for simulation."""

import Sofa

import os
import sys

import trimesh


import numpy as np
import glob

from sofa_utils import (  # noqa: E402
    add_header,
    add_solver,
    VolumeConfig,
    add_volume,
    add_cavity,
    SceneController,
)

FLAG_INVERSE = True


def load_stl_vertices(file_path):
    """
    Load an STL file and return vertex positions.

    Args:
        file_path (str): Path to the STL file

    Returns:
        numpy.ndarray: Array of vertex positions with shape (n_vertices, 3)
    """
    # Load the mesh
    mesh = trimesh.load_mesh(file_path)

    # Get vertex positions
    vertices = mesh.vertices

    return vertices


class InvSceneController(Sofa.Core.Controller):
    """GUI Controller for the SOFA Scene."""

    def __init__(self, root_node, chamber_list):
        """Initialize the SceneController."""
        Sofa.Core.Controller.__init__(self)
        self.root_node = root_node
        self.pressure_obj_list = []
        for chamber in chamber_list:
            pressure_obj = chamber.getObject('SPA')
            self.pressure_obj_list.append(pressure_obj)

    def onAnimateBeginEvent(self, event):
        """Read and set the values of the pressure and volume."""
        effector_delta = self.root_node.solver_node.effector_node.getObject(
            'effector'
        ).delta.value
        print(f'norm of effector delta: {np.linalg.norm(effector_delta)}')

        for i, chamber in enumerate(self.pressure_obj_list):
            value = chamber.pressure.value
            print(f'Chamber {i} pressure: {value}')
        print('----------------------')


def createScene(rootNode: Sofa.Core.Node) -> Sofa.Core.Node:
    """Set up the scene configuration."""
    add_header(root_node=rootNode, dt=0.01, inverse=FLAG_INVERSE)
    solver_node = add_solver(rootNode, iterative=True)

    mesh_path = './'
    volume_config = VolumeConfig(
        mesh_path=mesh_path + 'sphere_with_chambers.vtk',
        mass=20,
        youngs_modulus=1e5,
        poisson_ratio=0.48,
        box_dims=[
            [-5, -5, -5, 5, 5, 5]
        ],
        visu_mesh=mesh_path + 'sphere.stl',
    )

    volume_node = add_volume(
        parent_node=solver_node,
        config=volume_config,
    )
    i = 0
    chamber_node_list = []

    # Count how many chamber_{number}.stl files are in the mesh_path
    chamber_files = sorted(glob.glob(os.path.join(mesh_path, 'chamber_*.stl')))
    num_chambers = len(chamber_files)

    for i in range(num_chambers):
        chamber_node = add_cavity(
            parent_node=volume_node,
            cavity_name=f'chamber_{i}',
            cavity_model=mesh_path + f'chamber_{i}.stl',
            inverse=FLAG_INVERSE,
        )
        chamber_node_list.append(chamber_node)

    if FLAG_INVERSE:
        # First, sample points on the surface of the volume mesh by casting 
        # rays from the center of the volume mesh and finding intersections
        # with the surface mesh.
        volume_node.init()
        init_pos = volume_node.visu_node.getObject('visuModel').position.value
        init_pos_np = np.array(init_pos)
        num_points = min(100, len(init_pos_np))
        center = np.mean(init_pos_np, axis=0)
        # Generate num_points directions uniformly on the sphere
        phi = np.random.uniform(0, 2 * np.pi, num_points)
        costheta = np.random.uniform(-1, 1, num_points)
        theta = np.arccos(costheta)
        x = np.sin(theta) * np.cos(phi)
        y = np.sin(theta) * np.sin(phi)
        z = np.cos(theta)
        directions = np.stack([x, y, z], axis=1)

        # Prepare meshes for ray intersection
        volume_mesh = trimesh.load_mesh(mesh_path + 'sphere.stl')
        surface_mesh = trimesh.load_mesh('volume00001.stl')

        # For each direction, cast a ray from the center and find
        # intersections
        volume_locations, volume_ray_ids, _ = (
            volume_mesh.ray.intersects_location(
                ray_origins=np.tile(center, (num_points, 1)),
                ray_directions=directions,
            )
        )

        surface_locations, surface_ray_ids, _ = (
            surface_mesh.ray.intersects_location(
                ray_origins=np.tile(center, (num_points, 1)),
                ray_directions=directions,
            )
        )

        # Find rays that intersect both meshes
        common_ray_ids = np.intersect1d(volume_ray_ids, surface_ray_ids)

        # Get the first intersection for each common ray ID
        init_position_list = []
        target_pos_list = []
        sampled_ind = []

        for ray_id in common_ray_ids:
            # Find all intersections for this ray in both meshes
            volume_mask = volume_ray_ids == ray_id
            surface_mask = surface_ray_ids == ray_id

            # Get the first (closest) intersection for each mesh
            volume_hits = volume_locations[volume_mask]
            surface_hits = surface_locations[surface_mask]

            if len(volume_hits) > 0 and len(surface_hits) > 0:
                # Calculate distances from ray origin (center) to
                # find closest intersection
                volume_distances = np.linalg.norm(volume_hits - center, axis=1)
                surface_distances = np.linalg.norm(
                    surface_hits - center, axis=1
                )

                # Take the closest intersection for each mesh
                closest_volume_idx = np.argmin(volume_distances)
                closest_surface_idx = np.argmin(surface_distances)

                init_position_list.append(
                    volume_hits[closest_volume_idx].tolist()
                )
                target_pos_list.append(
                    surface_hits[closest_surface_idx].tolist()
                )
                sampled_ind.append(ray_id)

        # convert to numpy arrays and filter points
        sampled_indices = np.array(sampled_ind)
        init_position = np.array(init_position_list)
        target_pos = np.array(target_pos_list)
        init_distance = np.linalg.norm(init_position - target_pos, axis=1)
        # select only points with distance larger than 1
        # mask = init_distance > 1
        # select 5 most different points
        mask = np.argsort(init_distance)[-5:]
        init_position = init_position[mask]
        target_pos = target_pos[mask]
        # sampled_indices = sampled_indices[mask]
        sampled_indices = np.array(range(len(init_position)))
        print(
            f'Number of sampled points after filtering: {len(init_position)}'
        )

        effector_node = solver_node.addChild('effector_node')
        effector_node.addObject(
            'MechanicalObject',
            name='effector_mesh',
            position=init_position,
            showObject=True,
            showObjectScale=0.01,
        )

        effector_node.addObject(
            'PositionEffector',
            name='effector',
            indices=sampled_indices,
            effectorGoal=target_pos,
        )
        effector_node.addObject('BarycentricMapping', name='mapping')

        effector_visu = effector_node.addChild('effector_visu')
        effector_visu.addObject(
            'MechanicalObject',
            name='effector_visu_mo',
            position=target_pos,
            showObject=True,
            showObjectScale=5,
        )
        rootNode.addObject(
            InvSceneController(
                root_node=rootNode, chamber_list=chamber_node_list
            )
        )

    else:
        rootNode.addObject(SceneController(chamber_list=chamber_node_list))

    return rootNode
