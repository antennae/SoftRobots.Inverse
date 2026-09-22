"""
SmoothSlidingForceActuator: force localization on a soft sphere.

Same problem and same initial guess as SlidingForceActuator.py (see common.py).
The smooth variant is mathematically equivalent to SlidingForceActuator (it
stores the location as barycentric weights instead of tangent-plane
coordinates) and adds low-pass filtering of the sliding direction
(dirMomentum) and the sliding step (slideMomentum).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import (INIT_FORCE, INIT_THETA_PHI, RADIUS, MeasurementController,  # noqa: E402
                    addEstimate, addHeader, addTruth, closestTriangle, direction)


def createScene(rootNode):
    addHeader(rootNode)
    truth = addTruth(rootNode)
    estimate, force = addEstimate(rootNode, truth)

    initTriangle = closestTriangle(force.loader, RADIUS * direction(*INIT_THETA_PHI))
    actuator = force.addObject('SmoothSlidingForceActuator', name='actuator',
                               topology='@container', triangleIndices=[initTriangle],
                               initForce=[0, 0, INIT_FORCE],
                               maxStepSize=1.0, stepDamping=0.5,
                               dirMomentum=0.7, slideMomentum=0.5,
                               epsilonForce=1e-6, epsilonSliding=1e-6,
                               showForce=True, visuScale=10)
    force.addObject('BarycentricMapping')

    estimate.addObject(MeasurementController(name='controller', estimate=estimate, actuator=actuator))
    return rootNode
