#define SOFTROBOTS_INVERSE_AREACONTACTSLIDINGFORCEACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/AreaContactSlidingForceActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

int AreaContactSlidingForceActuatorClass = RegisterObject(
    "Area contact force actuator with sigmoid pressure distribution. "
    "Extends spherical sliding with a contact radius DOF. "
    "6 DOFs per contact: (px, py, pz, dTheta, dPhi, dR).")
.add< AreaContactSlidingForceActuator<Vec3Types> >(true)
;

template class SOFA_SOFTROBOTS_INVERSE_API AreaContactSlidingForceActuator<Vec3Types>;

} // namespace
