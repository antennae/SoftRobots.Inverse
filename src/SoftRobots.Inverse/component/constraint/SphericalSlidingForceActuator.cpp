#define SOFTROBOTS_INVERSE_SPHERICALSLIDINGFORCEACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/SphericalSlidingForceActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

int SphericalSlidingForceActuatorClass = RegisterObject(
    "Applies a force that slides across a mesh using spherical parameterization (S^Par). "
    "Replaces per-triangle barycentric sliding with globally continuous (theta, phi) coordinates, "
    "eliminating Jacobian discontinuities at triangle boundaries.")
.add< SphericalSlidingForceActuator<Vec3Types> >(true)
;

template class SOFA_SOFTROBOTS_INVERSE_API SphericalSlidingForceActuator<Vec3Types>;

} // namespace
