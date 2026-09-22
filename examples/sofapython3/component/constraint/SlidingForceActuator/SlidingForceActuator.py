"""
SlidingForceActuator: force localization on a soft sphere.

The actuator starts on a wrong triangle and slides over the surface until the
markers of `estimation` match the ones measured on `groundTruth` (see common.py).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import (INIT_FORCE, INIT_THETA_PHI, RADIUS, MeasurementController,  # noqa: E402
                    addEstimate, addHeader, addTruth, closestTriangle, direction)


def createScene(rootNode):
    addHeader(rootNode)
    truth = addTruth(rootNode)
    estimate, force = addEstimate(rootNode)

    initTriangle = closestTriangle(force.loader, RADIUS * direction(*INIT_THETA_PHI))
    actuator = force.addObject('SlidingForceActuator', name='actuator',
                               topology='@container', triangleIndices=[initTriangle],
                               initForce=[0, 0, INIT_FORCE],
                               maxStepSize=1.0, stepDamping=0.5,
                               epsilonForce=1e-6, epsilonSliding=1e-6,
                               showForce=True, visuScale=10)
    force.addObject('BarycentricMapping')

    rootNode.addObject(MeasurementController(name='controller', truth=truth, estimate=estimate, actuator=actuator))
    return rootNode
