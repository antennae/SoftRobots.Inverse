#define SOFTROBOTS_INVERSE_SURFACESLIDINGSENSOR_CPP

#include <SoftRobots.Inverse/component/constraint/SurfaceSlidingSensor.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;


int SurfaceSlidingSensorClass = RegisterObject("This component measures the distance between a point to a set of surfaces.")
.add< SurfaceSlidingSensor<Vec3Types> >(true)

;

template class SurfaceSlidingSensor<Vec3Types>;

} // namespace softrobotsinverse::constraint

