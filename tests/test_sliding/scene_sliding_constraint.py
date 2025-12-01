"""SOFA Scene for simulation using sliding constraints."""

import Sofa

import os
import sys

import numpy as np
import glob

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/utils'
)

from sofa_utils import (  # noqa: E402
    add_header,
    add_solver,
    VolumeConfig,
    add_volume,
    add_cavity,
    SceneController,
)

FLAG_INVERSE = True


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
        # effector_delta = self.root_node.solver_node.effector_node.getObject(
        #     'effector'
        # ).delta.value
        # print(f'norm of effector delta: {np.linalg.norm(effector_delta)}')

        for i, chamber in enumerate(self.pressure_obj_list):
            value = chamber.pressure.value
            print(f'Chamber {i} pressure: {value}')
        print('----------------------')


def createScene(rootNode: Sofa.Core.Node) -> Sofa.Core.Node:
    """Set up the scene configuration."""
    add_header(root_node=rootNode, dt=0.01, inverse=FLAG_INVERSE)
    solver_node = add_solver(rootNode, iterative=False)

    mesh_path = './'
    volume_config = VolumeConfig(
        mesh_path=mesh_path + 'sphere_with_chambers.vtk',
        mass=20,
        youngs_modulus=1e5,
        poisson_ratio=0.48,
        box_dims=[[-5, -5, -5, 5, 5, 5]],
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
        # original_mesh = trimesh.load_mesh(mesh_path + 'sphere.stl')
        # target_mesh = trimesh.load_mesh('volume00001.stl')

        target_mesh_node = solver_node.addChild('target_mesh_node')

        target_mesh_loader = target_mesh_node.addObject(
            'MeshSTLLoader',
            name='target_mesh_loader',
            filename='volume00001.stl',
        )
        target_mesh_node.addObject(
            'MechanicalObject',
            name='effector_mesh',
            position=target_mesh_loader.position,
            showObject=True,
            showObjectScale=5,
        )

        # target_mesh_node.addObject(
        #     'PositionEffector',
        #     name='effector',
        #     indices=sampled_indices,
        #     effectorGoal=target_pos,
        # )
        # target_mesh_node.addObject('BarycentricMapping', name='mapping')

        sliding_node = solver_node.addChild('sliding_node')
        mesh_loader = sliding_node.addObject(
            'MeshSTLLoader',
            name='mesh_loader',
            filename=mesh_path + 'sphere.stl',
        )
        sliding_node.addObject(
            'MechanicalObject',
            name='sliding_mesh',
            position=mesh_loader.position,
            showObject=True,
            showObjectScale=5,
        )

        sliding_node.addObject(
            'SurfaceSlidingEffector',
            name='sliding_constraint',
            surfaceState='@../target_mesh_node/effector_mesh',
            triangles=target_mesh_loader.triangles,
            pointIndex=np.arange(
                start=0, stop=len(mesh_loader.position.value), step=1
            ),
            limitShiftToTarget=True,
            maxShiftToTarget=0.1,
        )

        sliding_node.addObject(
            'BarycentricMapping',
            name='sliding_mapping',
        )

        rootNode.addObject(
            InvSceneController(
                root_node=rootNode, chamber_list=chamber_node_list
            )
        )

    else:
        rootNode.addObject(SceneController(chamber_list=chamber_node_list))

    return rootNode
