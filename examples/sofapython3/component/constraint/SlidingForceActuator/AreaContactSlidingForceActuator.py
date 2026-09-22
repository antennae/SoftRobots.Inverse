"""
AreaContactSlidingForceActuator: localization of a distributed load on a soft sphere.

Same problem and same initial guess as SphericalSlidingForceActuator.py (see
common.py), but the ground truth force is spread over a pressure patch. The
actuator estimates the pressure, the location of the patch and its radius.
The ground truth patch uses the sigmoid profile assumed by the actuator.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import (INIT_FORCE, INIT_THETA_PHI, MESH_DIR, SIGMOID_K, MeasurementController, addEstimate,  # noqa: E402
                    addHeader, addTruth, sparAngles)

TRUTH_CONTACT_RADIUS = 8.0
INIT_CONTACT_RADIUS = 5.0

AREA_FORCE = 100000
# This actuator works with a pressure: its initForce and currentForces are pressure * dt.
INIT_PRESSURE = AREA_FORCE / (np.pi * INIT_CONTACT_RADIUS ** 2)


def createScene(rootNode):
    addHeader(rootNode)
    truth = addTruth(rootNode, contactRadius=TRUTH_CONTACT_RADIUS)
    estimate, force = addEstimate(rootNode, truth)

    initTheta, initPhi = sparAngles(*INIT_THETA_PHI)
    actuator = force.addObject('AreaContactSlidingForceActuator', name='actuator',
                               topology='@container', sparFile=MESH_DIR + 'sphere.spar',
                               initTheta=[initTheta], initPhi=[initPhi], initRadius=[INIT_CONTACT_RADIUS],
                               initForce=[0, 0, INIT_PRESSURE],
                               maxStepSize=0.05, stepDamping=0.25,
                               dirMomentum=0.5, slideMomentum=0.25,
                               sigmoidK=SIGMOID_K, minRadius=2.0, maxRadius=25.0,
                               maxRadiusStep=1.0, radiusMomentum=0.25,
                               epsilonForce=1e-6, epsilonSliding=1e-5, epsilonRadius=1e-5,
                               showForce=True, visuScale=3)
    force.addObject('BarycentricMapping')

    estimate.addObject(MeasurementController(name='controller', estimate=estimate, actuator=actuator))
    return rootNode
