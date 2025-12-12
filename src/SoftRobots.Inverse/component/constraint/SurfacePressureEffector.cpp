#define SOFTROBOTS_INVERSE_SURFACEPRESSUREEFFECTOR_CPP

#include <SoftRobots.Inverse/component/config.h>
#include <SoftRobots.Inverse/component/constraint/SurfacePressureEffector.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint {

using namespace sofa::defaulttype;
using sofa::core::ConstraintParams;
using namespace sofa::core;

////////////////////////////////////////////    FACTORY
/////////////////////////////////////////////////
using namespace sofa::helper;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int SurfacePressureEffectorClass =
    sofa::core::RegisterObject(
        "This component is used to describe the desired pressure of a cavity "
        "by applying an imposed pressure on surfaces  ")
        .add<SurfacePressureEffector<sofa::defaulttype::Vec3Types>>(true);
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related
// type. This goes with the extern template declaration in the .h. Declaring
// extern template avoid the code generation of the template for each
// compilation unit. see:
// http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_INVERSE_API
    SurfacePressureEffector<sofa::defaulttype::Vec3Types>;

} // namespace softrobotsinverse::constraint
