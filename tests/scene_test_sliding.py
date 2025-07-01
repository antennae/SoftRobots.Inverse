import Sofa
import Sofa.Core


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
                    Sofa.GL.Component.Shader\
                    Sofa.Component.Engine.Analyze\
                        Sofa.Component.Mapping.NonLinear\
                            Sofa.Component.LinearSolver.Direct'


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
            epsilon=0.001,
            maxIterations=1000,
            tolerance=1e-10,
            multithreading=True,
            objective=1,
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
            iterations=1000,
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


def createScene(rootNode: Sofa.Core.Node) -> Sofa.Core.Node:
    """Set up the scene configuration."""
    add_header(root_node=rootNode, dt=0.01, inverse=True)
    solver_node = add_solver(rootNode, iterative=False)

    tri_position = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
    surfaces_mesh = solver_node.addObject(
        'MeshTopology',
        name='triangles_mesh',
        position=tri_position,
        triangles=[[0, 1, 2], [0, 2, 3]],
    )
    solver_node.addObject(
        'MechanicalObject',
        name='triangles_mo',
        template='Vec3d',
        showObject=True,
        showObjectScale=5,
    )

    solver_node.addObject(
        'UniformMass',
        name='mass',
        totalMass=1.0,
    )

    c_node = solver_node.addChild('constraint_node')

    position_list = [
        [0.5, -0.5, 0.5],
        [0.3, 0.3, 0.5],
        [0.3, 0.3, -0.5],
        [0, 0, 0.1],
    ]

    c_node.addObject(
        'MechanicalObject',
        name='point',
        # position=[[0.5, -0.5, 0.5]],
        position=position_list,
        template='Vec3d',
        showObject=True,
        showObjectScale=10,
    )

    c_node.addObject(
        'UniformMass',
        name='mass',
        totalMass=0.5,
    )

    c_node.addObject(
        'SurfaceSlidingEffector',
        name='slidingConstraint',
        surfaceState='@../triangles_mo',  # postions of target surface
        pointIndex=[0, 1, 2, 3],  # indices of points to be constrained
        triangles=surfaces_mesh.triangles,  # triangles of target surface
    )

    f_node = c_node.addChild('force_node')
    f_node.addObject(
        'MechanicalObject',
        name='force_mo',
        position=[
            [0.5, -0.5, 0.5],
            [0.3, 0.3, 0.5],
            [0.3, 0.3, -0.5],
            [0, 0, 0.1],
        ],
        template='Vec3d',
    )
    for i in range(4):
        f_node.addObject(
            'ForcePointActuator',
            indices=[i],  # , 1, 2, 3],  # indices of points to be actuated
            name=f'forcePointActuator_{i}',
        )
    f_node.addObject('BarycentricMapping', name='mapping')
