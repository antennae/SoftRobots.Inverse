#define SOFTROBOTS_INVERSE_FORCELOCALIZATIONACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/ForceLocalizationActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

int ForceLocalizationActuatorClass = RegisterObject("This component is used to solve an inverse problem by estimating forces on a set of candidate points.")
.add< ForceLocalizationActuator<Vec3Types> >(true)
.add< ForceLocalizationActuator<Rigid3Types> >()
;

template class SOFA_SOFTROBOTS_INVERSE_API ForceLocalizationActuator<Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API ForceLocalizationActuator<Rigid3Types>;

} // namespace
