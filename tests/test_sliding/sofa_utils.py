"""
Utilies for SOFA scene

Author: Sizhe Tian
Date: 2025-05-22
"""

from dataclasses import dataclass, field
from typing import List, Optional

import Sofa
import Sofa.Core

import numpy as np

import tkinter as tk
from threading import Thread
from tkinter import ttk
import queue


REQUIRED_PLUGINS = 'SoftRobots.Inverse  \
                    SoftRobots  \
                    SOFA.Component.IO.Mesh  \
                    SOFA.Component.Engine.Generate\
                    SOFA.Component.Mass \
                    SOFA.Component.LinearSolver.Direct \
                    SOFA.Component.Constraint.Lagrangian.Correction \
                    Sofa.GL.Component.Rendering3D  \
                    Sofa.Component.Diffusion  \
                    Sofa.Component.AnimationLoop \
                    Sofa.Component.Collision.Geometry\
                    Sofa.Component.Constraint.Lagrangian.Solver\
                    Sofa.Component.Engine.Select \
                    Sofa.Component.LinearSolver.Iterative\
                    Sofa.Component.ODESolver.Backward  \
                    Sofa.Component.Setting  \
                    Sofa.Component.SolidMechanics.FEM.Elastic  \
                    Sofa.Component.SolidMechanics.Spring\
                    Sofa.Component.StateContainer \
                    Sofa.Component.Topology.Container.Dynamic \
                    Sofa.Component.Topology.Container.Grid  \
                    Sofa.Component.Visual \
                    Sofa.Component.Topology.Container.Constant\
                    Sofa.Component.Engine.Transform \
                    Sofa.Component.Mapping.Linear \
                    Sofa.Component.MechanicalLoad \
                    Sofa.Component.Constraint.Lagrangian.Correction\
                    Sofa.Component.IO.Mesh\
                    Sofa.Component.LinearSolver.Direct\
                    Sofa.Component.Mass\
                    Sofa.GL.Component.Shader'


VISUAL_STYLE_FLAGS = 'showVisualModels \
            hideBehaviorModels showCollisionModels \
            hideBoundingCollisionModels showForceFields \
                showInteractionForceFields hideWireframe'


class SceneConfig:
    """Class to store all configuration for the SOFA scene."""

    def __init__(
        self,
        gravity=(0, 0, 0),
        time_step=0.01,
        collision_detection=False,
        mesh_constraint_type='spring',
        mesh_file=None,
    ):
        """Initialize the SceneConfig."""
        # Validate inputs
        if not isinstance(gravity, tuple) or len(gravity) != 3:
            raise ValueError('Gravity must be a tuple of three floats.')
        if time_step <= 0:
            raise ValueError('Time step must be positive.')

        self.gravity = gravity
        self.time_step = time_step
        self.collision_detection = collision_detection
        self.mesh_constraint_type = mesh_constraint_type
        self.mesh_file = mesh_file

        self.required_plugin_list = REQUIRED_PLUGINS
        self.visual_style_flags = VISUAL_STYLE_FLAGS


def add_header(root_node, dt, inverse=True):
    """
    Add header objects to the root node of the scene.

    Parameters:
    - root_node: The root node of the scene.
    - dt: The time step value in seconds.
    - inverse: A boolean value to determine whether to use
            QPInverseProblemSolver or the GenericConstraintSolver.

    Returns:
    None
    """
    config = SceneConfig()
    root_node.addObject(
        'RequiredPlugin', pluginName=config.required_plugin_list
    )
    root_node.addObject('VisualStyle')
    root_node.addObject(
        'FreeMotionAnimationLoop',
        name='AnimationLoop',
        parallelCollisionDetectionAndFreeMotion=True,
        parallelODESolving=True,
    )
    if not inverse:
        root_node.addObject(
            'GenericConstraintSolver',
            name='ConstraintSolver',
            tolerance=1e-3,
            maxIterations=2000,
            multithreading=True,
        )
    else:
        root_node.addObject(
            'QPInverseProblemSolver',
            name='ConstraintSolver',
            printLog=0,
            epsilon=0.01,
            maxIterations=10000,
            tolerance=1e-5,
            multithreading=True,
            objective=10,
        )

    root_node.VisualStyle.displayFlags = config.visual_style_flags
    root_node.findData('gravity').value = config.gravity
    root_node.findData('dt').value = dt

    root_node.addObject('BackgroundSetting', color='0.8 0.8 0.8')
    root_node.addObject('LightManager')
    root_node.addObject(
        'PositionalLight',
        name='light1',
        color='0.8 0.8 0.8',
        position='0 60 -50',
    )
    root_node.addObject(
        'PositionalLight',
        name='light2',
        color='0.8 0.8 0.8',
        position='0 -60 50',
    )


def add_solver(root_node, iterative=True):
    """
    Add a solver node to the root node of the scene.

    Args:
        root_node (Node): The root node of the scene.
        iterative (bool): A boolean value to determine whether to use
                        CGLinearSolver or the SparseLDLSolver.
    """
    solver_node = root_node.addChild('solver_node')

    solver_node.addObject(
        'EulerImplicitSolver',
        firstOrder=False,
        rayleighStiffness=0.05,
        rayleighMass=0.02,
        vdamping=0,
    )
    if iterative:
        solver_node.addObject(
            'CGLinearSolver',
            name='Solver',
            template='CompressedRowSparseMatrixd',
            threshold=1e-5,
            iterations=100,
            tolerance=1e-5,
        )
    else:
        solver_node.addObject(
            'SparseLDLSolver',
            name='Solver',
            template='CompressedRowSparseMatrixd',
        )

    solver_node.addObject(
        'GenericConstraintCorrection',
        # name='ConstraintCorrection',
        # linearSolver=solver_node.Solver.getLinkPath(),
    )

    return solver_node


@dataclass
class RegionalStiffeningConfig:
    """
    Dataclass to hold regional stiffening configuration parameters.

    Args:
        region (list): The region to stiffen.
        modulus (float): The stiffness modulus.
    """

    region: List[List[float]]
    modulus: float


@dataclass
class VolumeConfig:
    """
    Dataclass to hold volume configuration parameters.

    Args:
        mesh_path (str): The path to the mesh file for the volume.
        mass (float): The mass of the volume.
        youngs_modulus (float): The Young's modulus of the volume.
        poisson_ratio (float): The Poisson's ratio of the volume.
        box_dims (list): The dimensions of the box to be fixed.
        visu_mesh (str, optional): The path to the mesh file for visualization.

    """

    mesh_path: str
    mass: float = 1.0
    youngs_modulus: float = 1000.0
    poisson_ratio: float = 0.3
    box_dims: List[List[float]] = field(default_factory=list)
    visu_mesh: Optional[str] = None
    regional_stiffening: Optional[RegionalStiffeningConfig] = None


def add_volume(
    parent_node: Sofa.Core.Node,
    config: VolumeConfig,
) -> Sofa.Core.Node:
    """
    Add a volume to the node.

    Args:
        parent_node (Node): The parent node.
        config (VolumeConfig): The volume configuration.
    """
    volume_node = parent_node
    volume_node.addObject(
        'MeshVTKLoader',
        name='VolumeMeshLoader',
        filename=config.mesh_path,
    )
    volume_node.addObject(
        'MeshTopology', name='topology', src='@VolumeMeshLoader'
    )
    volume_node.addObject(
        'MechanicalObject',
        name='states',
        # template='CudaVec3f',
    )
    volume_node.init()
    volume_node.addObject('UniformMass', totalMass=config.mass)  # "kg"

    ym_vector = _get_youngs_modulus_vector(
        volume_node=volume_node,
        stiffen_config=config.regional_stiffening,
        youngs_modulus=config.youngs_modulus,
    )

    volume_node.addObject(
        'TetrahedronFEMForceField',
        # 'ParallelTetrahedronFEMForceField',
        name='volume',
        # youngModulus=youngs_modulus,  # "Pa"
        youngModulus=ym_vector,
        poissonRatio=config.poisson_ratio,
    )

    _add_fixed_box(volume_node, config)

    _add_visualization(volume_node, config)

    return volume_node


def _get_youngs_modulus_vector(
    volume_node: Sofa.Core.Node,
    stiffen_config: Optional[RegionalStiffeningConfig],
    youngs_modulus: float,
) -> np.ndarray:
    """
    Construct the Young's modulus vector.

    Args:
        volume_node (Node): The volume node.
        stiffen_config (list): The region to stiffen.
        youngs_modulus (float): The Young's modulus.

    """
    ym_vector = np.ones(volume_node.states.size.value) * youngs_modulus

    i = 0
    if stiffen_config is not None:
        for stiffen_dim in stiffen_config.region:
            stiffen = volume_node.addObject(
                'BoxROI',
                name=f'Stiffen_{i}',
                box=stiffen_dim,
                drawBoxes=True,
            )
            stiffen.init()
            ym_vector[stiffen.indices.value] = stiffen_config.modulus
            i += 1
    return ym_vector


def _add_fixed_box(volume_node: Sofa.Core.Node, config: VolumeConfig):
    """
    Add fixed box to the volume.

    Args:
        volume_node (Node): The volume node.
        config (VolumeConfig): The volume configuration.
    """
    if config.box_dims:
        i = 0
        for box_dim in config.box_dims:
            # check box_dim is of length 6
            assert len(box_dim) == 6, 'box_dim must be of length 6'
            box = volume_node.addObject(
                'BoxROI',
                name=f'Box{i}',
                box=box_dim,
                drawBoxes=True,
            )
            box.init()
            # check BoxROI contains nodes
            assert (
                len(box.indices.value) > 0
            ), 'Fixed box BoxROI must contain nodes'
            volume_node.addObject(
                'RestShapeSpringsForceField',
                name=f'FixBox{i}',
                points=box.indices.linkpath,
                stiffness=1e8,
                drawSpring=True,
            )
            i = i + 1


def _add_visualization(volume_node: Sofa.Core.Node, config: VolumeConfig):
    """
    Add visualization to the volume.

    Args:
        volume_node (Node): The volume node.
        config (VolumeConfig): The volume configuration.
    """
    if config.visu_mesh:
        visu_node = volume_node.addChild('visu_node')
        visu_node.addObject(
            'MeshSTLLoader',
            filename=config.visu_mesh,
            name='visualMeshLoader',
        )
        visu_node.addObject(
            'OglModel',
            name='visuModel',
            src='@visualMeshLoader',
        )
        visu_node.addObject('BarycentricMapping')
        # stl_exporter = visu_node.addObject(
        #     'STLExporter',
        #     exportAtBegin=True,
        #     filename='volume',
        #     exportEveryNumberOfSteps=1000,
        # )

        # visu_node.addObject(
        #     'VolumeFromTriangles',
        #     name='volume',
        #     update=True,
        #     positions='@visuModel.position',
        #     triangles='@visuModel.triangles',
        # )


class SceneController(Sofa.Core.Controller):
    """GUI Controller for the SOFA Scene."""

    def __init__(self, chamber_list):
        """Initialize the SceneController."""
        Sofa.Core.Controller.__init__(self)

        self.pressure_obj_list = []
        for chamber in chamber_list:
            pressure_obj = chamber.getObject('SPC')
            self.pressure_obj_list.append(pressure_obj)
        self.pressure_list = [0.0] * len(chamber_list)
        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()
        self.thread = Thread(target=self.threaded_function)
        self.thread.daemon = True
        self.thread.start()

    def threaded_function(self):
        """Threaded function to create GUI."""
        window = tk.Tk()
        window.title('Pressure Controller')

        # Style Configuration
        style = ttk.Style()
        style.configure('TLabel', font=('Arial', 14), background='#f0f0f0')
        style.configure('TEntry', font=('Arial', 12))
        style.configure(
            'TButton',
            font=('Arial', 12),
            background='#4CAF50',
            foreground='white',
        )

        # Add the pressure inputs
        entries = []
        for i, pressure in enumerate(self.pressure_list):
            label = ttk.Label(window, text=f'Pressure {i + 1} (Pa)')
            label.grid(row=i, column=0, padx=10, pady=10)
            entry = ttk.Entry(window)
            entry.insert(0, f'{pressure:.2f}')
            entry.grid(row=i, column=1, padx=10, pady=10)
            entries.append(entry)

        def update_pressures():
            for i, entry in enumerate(entries):
                try:
                    value = float(entry.get())
                    self.output_queue.put((i, value))
                except ValueError:
                    pass  # Handle invalid input gracefully

        update_button = ttk.Button(
            window, text="Update Pressures", command=update_pressures
        )
        update_button.grid(
            row=len(self.pressure_list), column=0, columnspan=2, pady=10
        )

        window.mainloop()

    def onAnimateBeginEvent(self, event):
        """Read and set the values of the pressure and volume."""
        while not self.output_queue.empty():
            i, value = self.output_queue.get()
            # self.pressure_list[i] = value
            self.pressure_obj_list[i].value = [value]


def fix_base_position(node, base_box_roi=[-8, -8, -1, 8, 8, 1]):
    """Fix the base position of the robot."""
    node.init()
    base_box = node.addObject(
        'BoxROI',
        name='boxROI_base',
        orientedBox=base_box_roi,
        drawBoxes=True,
        strict=False,
        drawTetrahedra=False,
    )  # if auto-complete, set 8 depending on the robot's dimensions
    base_box.init()
    # print('selected : ')
    # print(base_box.indices.value)
    node.addObject(
        'RestShapeSpringsForceField',
        points=base_box.indices.value,
        angularStiffness=1e10,
        stiffness=1e10,
    )  # to anchor the robot's base in space


def add_cavity(
    parent_node: Sofa.Core.Node,
    cavity_name: str,
    cavity_model: str,
    inverse: bool = False,
) -> tuple:
    """
    Add a cavity to the scene.

    Args:
        parent (Sofa.Core.Node): The parent node.
        name_c (str): The name of the cavity.
        cavity_model (str): The model of the cavity (stl file).
        cylinder_params (dict): The parameters of the cylinder.
        # pressure_type (str, optional): the type of pressure implementation.

    Returns:
        Sofa.Core.Node: The cavity node.
    """
    chamber_node = parent_node.addChild(cavity_name)
    chamber_node.addObject(
        'MeshSTLLoader', name='MeshLoader', filename=cavity_model
    )
    chamber_node.addObject('MeshTopology', name='container', src='@MeshLoader')
    chamber_node.addObject(
        'MechanicalObject',
        name='dofs',
    )
    if inverse:
        chamber_node.addObject(
            'SurfacePressureActuator',
            name='SPA',
        )
    else:
        chamber_node.addObject(
            'SurfacePressureConstraint',
            name='SPC',
            value=0,
            valueType='pressure',
        )

    chamber_node.addObject('BarycentricMapping')

    return chamber_node
