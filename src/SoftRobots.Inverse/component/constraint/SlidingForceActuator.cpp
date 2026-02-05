#define SOFTROBOTS_INVERSE_SLIDINGFORCEACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/SlidingForceActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

int SlidingForceActuatorClass = RegisterObject("This component applies a force on a surface and optimizes its location (Cartesian local coordinates).")
.add< SlidingForceActuator<Vec3Types> >(true)
;

template class SOFA_SOFTROBOTS_INVERSE_API SlidingForceActuator<Vec3Types>;

} // namespace