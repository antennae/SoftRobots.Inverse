"""
Shared setup for the sliding force actuator examples.

Each example solves the same force localization problem on a soft sphere:

  - `groundTruth` is a forward simulation. A known point force pushes on the sphere
    and a set of surface markers records the resulting displacement.
  - `estimation` is the inverse simulation. PositionEffectors pull the same
    markers toward the positions measured on `groundTruth`, and a sliding force
    actuator has to find where the force is applied and how large it is.

The two simulations are independent: each has its own animation loop and
constraint solver, and the root `AnimationLoopParallelScheduler` steps them in
parallel. After every step, `DataExchange` components copy the marker and load
positions of `groundTruth` into `estimation`, where they drive the effector
goals. A controller in `estimation` reports the localization error.

The two spheres are drawn side by side (`groundTruth` is shifted by TRUTH_OFFSET).

Units: mm, kg, s (forces in mN, Young's modulus in kPa).
"""

import os

import numpy as np
import Sofa

MESH_DIR = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

RADIUS = 30.0
TRUTH_OFFSET = np.array([80.0, 0.0, 0.0])

# Ground truth load: location given as (theta, phi) in degrees, theta measured
# from +Z. The force pushes along the inward normal.
TRUTH_THETA_PHI = (35.0, 20.0)
TRUTH_FORCE = 2500.0

# Initial guess of the actuators. The force guess is a constraint multiplier (force * dt), like every
# force Data of the actuators (initForce, minForce, maxForce, currentForces).
INIT_THETA_PHI = (60.0, 70.0)
INIT_FORCE = 100.0 * 0.01

NB_MARKERS = 16

# Sigmoid sharpness (1/mm) of the pressure patch of the area contact example: C(d, r) = 1 / (1 + exp(k (d - r)))
SIGMOID_K = 3.0


def direction(theta_deg, phi_deg):
    """Unit vector for spherical angles (theta from +Z), in degrees."""
    theta, phi = np.radians(theta_deg), np.radians(phi_deg)
    return np.array([np.sin(theta) * np.cos(phi),
                     np.sin(theta) * np.sin(phi),
                     np.cos(theta)])


def sparAngles(theta_deg, phi_deg):
    """(theta, phi) in radians, in the frame of mesh/sphere.spar.

    sphere.spar was built with `--pole 0,0,-1`: the parameterization is singular at its poles, so the
    theta = 0 pole is put on the fixed base where the actuator never slides. build_spherical_param.py
    maps that direction to +Z with a half turn around X, which is reproduced here.
    """
    x, y, z = direction(theta_deg, phi_deg)
    return float(np.arccos(-z)), float(np.arctan2(-y, x) % (2 * np.pi))


def markerPositions(n=NB_MARKERS, zMin=-0.5):
    """Fibonacci points on the sphere above the fixed base, slightly inside the surface."""
    golden = np.pi * (3.0 - np.sqrt(5.0))
    z = np.linspace(0.95, zMin, n)
    r = np.sqrt(1.0 - z * z)
    phi = golden * np.arange(n)
    return 0.97 * RADIUS * np.stack([r * np.cos(phi), r * np.sin(phi), z], axis=1)


def closestTriangle(loader, point):
    """Index of the triangle of `loader` whose centroid is closest to `point`."""
    positions = np.array(loader.position.value)
    triangles = np.array(loader.triangles.value)
    centroids = positions[triangles].mean(axis=1)
    return int(np.argmin(np.linalg.norm(centroids - point, axis=1)))


def addHeader(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='MultiThreading')  # Needed to use components [AnimationLoopParallelScheduler,DataExchange]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [LinearSolverConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver')  # Needed to use components [NNCGConstraintSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Transform')  # Needed to use components [TransformEngine]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')  # Needed to use components [FixedProjectiveConstraint]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshSTLLoader,MeshVTKLoader]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.MechanicalLoad')  # Needed to use components [ConstantForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')  # Needed to use components [OglModel]

    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels '
                                                   'hideForceFields showInteractionForceFields')
    rootNode.gravity = [0, 0, 0]
    rootNode.dt = 0.01

    # Steps every child node that owns an animation loop, in parallel, then fires the DataExchange components.
    rootNode.addObject('AnimationLoopParallelScheduler')


def addSphere(parentNode, name, translation=(0.0, 0.0, 0.0), color=(0.7, 0.5, 0.5, 1.0)):
    """Soft sphere fixed at its base (-Z cap)."""
    translation = list(translation)
    sphere = parentNode.addChild(name)
    sphere.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    sphere.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    sphere.addObject('MeshVTKLoader', name='loader', filename=MESH_DIR + 'sphere.vtk', translation=translation)
    sphere.addObject('MeshTopology', src='@loader', name='container')
    sphere.addObject('MechanicalObject', name='dofs')
    sphere.addObject('UniformMass', totalMass=0.11)
    sphere.addObject('TetrahedronFEMForceField', poissonRatio=0.45, youngModulus=60)
    box = [translation[0] - RADIUS, translation[1] - RADIUS, translation[2] - RADIUS - 1,
           translation[0] + RADIUS, translation[1] + RADIUS, translation[2] - 0.75 * RADIUS]
    sphere.addObject('BoxROI', name='base', box=box, drawBoxes=False)
    sphere.addObject('FixedProjectiveConstraint', indices='@base.indices')
    sphere.addObject('LinearSolverConstraintCorrection')

    visu = sphere.addChild('visu')
    visu.addObject('MeshSTLLoader', name='loader', filename=MESH_DIR + 'sphere.stl', translation=translation)
    visu.addObject('OglModel', src='@loader', color=list(color))
    visu.addObject('BarycentricMapping')
    return sphere


def addMarkers(sphere, translation=(0.0, 0.0, 0.0), color=(0.0, 1.0, 0.0, 1.0)):
    markers = sphere.addChild('markers')
    markers.addObject('MechanicalObject', name='dofs', position=(markerPositions() + np.array(translation)).tolist(),
                      showObject=True, showObjectScale=1, drawMode=1, showColor=list(color))
    markers.addObject('BarycentricMapping')
    return markers


def addTruth(rootNode, contactRadius=None):
    """Forward simulation: the sphere loaded by the ground truth force.

    By default the load is a point force. With `contactRadius`, the same total force is spread over a
    pressure patch of that radius, with the sigmoid profile assumed by AreaContactSlidingForceActuator.
    The first point of `load.dofs` is always the center of the load.
    """
    scene = rootNode.addChild('groundTruth')
    scene.addObject('FreeMotionAnimationLoop')
    scene.addObject('NNCGConstraintSolver', tolerance=1e-6, maxIterations=1000)
    truth = addSphere(scene, 'sphere', translation=TRUTH_OFFSET, color=(0.5, 0.6, 0.7, 1.0))
    addMarkers(truth, translation=TRUTH_OFFSET)

    normal = direction(*TRUTH_THETA_PHI)
    center = 0.99 * RADIUS * normal + TRUTH_OFFSET
    if contactRadius is None:
        positions = [center]
        forces = [-TRUTH_FORCE * normal]
    else:
        vertices = np.array(truth.visu.loader.position.value)
        triangles = vertices[np.array(truth.visu.loader.triangles.value)]
        centroids = triangles.mean(axis=1)
        areas = 0.5 * np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                              triangles[:, 2] - triangles[:, 0]), axis=1)
        distances = np.linalg.norm(centroids - RADIUS * normal - TRUTH_OFFSET, axis=1)
        weights = areas / (1.0 + np.exp(SIGMOID_K * (distances - contactRadius)))
        patch = distances < contactRadius + 3.0 / SIGMOID_K
        pressure = TRUTH_FORCE / weights[patch].sum()
        positions = [center] + list(0.99 * (centroids[patch] - TRUTH_OFFSET) + TRUTH_OFFSET)
        forces = [np.zeros(3)] + [-pressure * w * normal for w in weights[patch]]

    load = truth.addChild('load')
    load.addObject('MechanicalObject', name='dofs', position=np.array(positions).tolist(),
                   showObject=True, showObjectScale=2.0, drawMode=1, showColor=[1, 0, 0, 1])
    load.addObject('ConstantForceField', indices=list(range(len(positions))), forces=np.array(forces).tolist(),
                   showArrowSize=0.02)
    load.addObject('BarycentricMapping')
    return scene


def addEstimate(rootNode, truth):
    """Inverse simulation: the sphere whose markers are driven toward the positions measured on `truth`.

    Returns the scene node and the node in which the sliding actuator has to be added. That node holds
    the surface mesh (topology + MechanicalObject) the actuator slides on.
    """
    estimate = rootNode.addChild('estimation')
    estimate.addObject('FreeMotionAnimationLoop')
    estimate.addObject('QPInverseProblemSolver', epsilon=1e-3, tolerance=1e-6, maxIterations=1000)

    # Copies of the ground truth data, filled by DataExchange after each step. TransformEngine removes the
    # display offset so the copies are expressed in the frame of the estimation sphere.
    measured = estimate.addChild('measured')
    for name in ['markers', 'load']:
        node = measured.addChild(name)
        node.addObject('MechanicalObject', name='dofs', position=truth.sphere.getChild(name).dofs.position.value)
        node.addObject('TransformEngine', name='here', input_position='@dofs.position',
                       translation=(-TRUTH_OFFSET).tolist())
        rootNode.addObject('DataExchange', name='exchange_' + name, template='vector<Vec3d>',
                           **{'from': f'@groundTruth/sphere/{name}/dofs.position',
                              'to': f'@estimation/measured/{name}/dofs.position'})

    sphere = addSphere(estimate, 'sphere')
    markers = addMarkers(sphere, color=(1.0, 1.0, 0.0, 1.0))
    markers.addObject('PositionEffector', name='effector', indices=list(range(NB_MARKERS)),
                      effectorGoal='@../../measured/markers/here.output_position')

    force = sphere.addChild('force')
    force.addObject('MeshSTLLoader', name='loader', filename=MESH_DIR + 'sphere.stl')
    force.addObject('MeshTopology', name='container', src='@loader')
    force.addObject('MechanicalObject', name='dofs')
    return estimate, force


class MeasurementController(Sofa.Core.Controller):
    """Reports the localization error of the actuator. Add it to the `estimation` node."""

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.estimate = kwargs['estimate']
        self.actuator = kwargs['actuator']
        self.printEvery = kwargs.get('printEvery', 20)
        self.step = 0

    def locationError(self):
        truthLocation = np.array(self.estimate.measured.load.here.output_position.value)[0]
        location = np.array(self.actuator.currentLocation.value)[0]
        return float(np.linalg.norm(location - truthLocation))

    def onAnimateEndEvent(self, event):
        self.step += 1
        if self.printEvery and (self.step == 1 or self.step % self.printEvery == 0):
            # The actuator stores the constraint multiplier, which is force * dt with EulerImplicitSolver.
            dt = self.getContext().getRoot().dt.value
            force = np.linalg.norm(np.array(self.actuator.currentForces.value)[0]) / dt
            radius = self.actuator.findData('currentRadius')
            if radius is not None:
                # Area contact: currentForces is a pressure, integrate it over the sigmoid patch.
                radius = float(np.array(radius.value).flatten()[0])
                force *= self.patchArea(radius)
            message = (f'[{self.actuator.getClassName()}] step {self.step}: '
                       f'location error = {self.locationError():.2f} mm, force = {force:.1f} mN')
            if radius is not None:
                message += f', contact radius = {radius:.2f} mm'
            print(message)

    def patchArea(self, radius):
        surface = self.estimate.sphere.force
        triangles = np.array(surface.dofs.position.value)[np.array(surface.container.triangles.value)]
        areas = 0.5 * np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                              triangles[:, 2] - triangles[:, 0]), axis=1)
        distances = np.linalg.norm(triangles.mean(axis=1) - np.array(self.actuator.currentLocation.value)[0], axis=1)
        return float(np.sum(areas / (1.0 + np.exp(SIGMOID_K * (distances - radius)))))
