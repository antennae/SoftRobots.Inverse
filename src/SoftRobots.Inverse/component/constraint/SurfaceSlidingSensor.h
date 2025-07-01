#pragma once

#include <SoftRobots.Inverse/component/behavior/Sensor.h>
#include <SoftRobots/component/constraint/SurfaceSlidingConstraint.h>
#include <sofa/core/behavior/ConstraintResolution.h>

#include <SoftRobots.Inverse/component/constraint/SurfacePressureSensor.h>

namespace softrobotsinverse::constraint {

using sofa::core::ConstraintParams;
using sofa::core::VecCoordId;
using sofa::core::behavior::ConstraintResolution;
using sofa::helper::ReadAccessor;
using softrobotsinverse::behavior::Sensor;
using softrobotsinverse::constraint::DoNothingConstraintResolution;



template <class DataTypes>
class SurfaceSlidingSensor
    : public Sensor<DataTypes>,
      public softrobots::constraint::SurfaceSlidingConstraint<DataTypes> {
public:
  SOFA_CLASS(SOFA_TEMPLATE(SurfaceSlidingSensor, DataTypes),
             SOFA_TEMPLATE(Sensor, DataTypes));

  typedef typename sofa::core::behavior::MechanicalState<DataTypes>
      MechanicalState;

  SurfaceSlidingSensor(MechanicalState *object = nullptr);
  ~SurfaceSlidingSensor() override;

  void getConstraintResolution(std::vector<ConstraintResolution *> &resTab,
                               unsigned int &offset) override;

private:
  using softrobots::constraint::SurfaceSlidingConstraint<
      DataTypes>::d_pointIndex;
  using softrobots::constraint::SurfaceSlidingConstraint<DataTypes>::m_distance;
};

#if !defined(SOFTROBOTS_INVERSE_SURFACESLIDINGSENSOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API
    SurfaceSlidingSensor<sofa::defaulttype::Vec3Types>;
#endif

} // namespace softrobotsinverse::constraint