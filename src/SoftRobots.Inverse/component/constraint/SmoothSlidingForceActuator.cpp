#define SOFTROBOTS_INVERSE_SMOOTHSLIDINGFORCEACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/SmoothSlidingForceActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

int SmoothSlidingForceActuatorClass = RegisterObject("This component applies a force that slides using SMOOTH vertex normals to avoid chattering at edges.")
.add< SmoothSlidingForceActuator<Vec3Types> >(true)
;

template class SOFA_SOFTROBOTS_INVERSE_API SmoothSlidingForceActuator<Vec3Types>;

} // namespace
