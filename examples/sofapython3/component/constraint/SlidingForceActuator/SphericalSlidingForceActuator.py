"""
SphericalSlidingForceActuator: force localization on a soft sphere.

Same problem and same initial guess as SlidingForceActuator.py (see common.py).
The location is parameterized by two global angles (theta, phi) on the
spherical parameterization of the surface, instead of local coordinates in the
current triangle. It requires a .spar file of the surface mesh:

    python build_spherical_param.py --mesh mesh/sphere.stl --pole 0,0,-1
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import (INIT_FORCE, INIT_THETA_PHI, MESH_DIR, MeasurementController, addEstimate, addHeader,  # noqa: E402
                    addTruth, sparAngles)


def createScene(rootNode):
    addHeader(rootNode)
    truth = addTruth(rootNode)
    estimate, force = addEstimate(rootNode)

    initTheta, initPhi = sparAngles(*INIT_THETA_PHI)
    actuator = force.addObject('SphericalSlidingForceActuator', name='actuator',
                               topology='@container', sparFile=MESH_DIR + 'sphere.spar',
                               initTheta=[initTheta], initPhi=[initPhi],
                               initForce=[0, 0, INIT_FORCE],
                               maxStepSize=0.02, stepDamping=0.25,
                               dirMomentum=0.5, slideMomentum=0.5,
                               epsilonForce=1e-6, epsilonSliding=1e-5,
                               showForce=True, visuScale=10)
    force.addObject('BarycentricMapping')

    rootNode.addObject(MeasurementController(name='controller', truth=truth, estimate=estimate, actuator=actuator))
    return rootNode
