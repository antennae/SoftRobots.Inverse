#pragma once

#include <SoftRobots.Inverse/component/behavior/Effector.h>
#include <SoftRobots/component/constraint/SurfaceSlidingConstraint.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace softrobotsinverse::constraint {

using sofa::core::ConstraintParams;
using sofa::core::VecCoordId;
using sofa::core::behavior::ConstraintResolution;
using sofa::helper::ReadAccessor;
using softrobotsinverse::behavior::Effector;

template <class DataTypes>
class SurfaceSlidingEffector
    : public Effector<DataTypes>,
      public softrobots::constraint::SurfaceSlidingConstraint<DataTypes> {
public:
  SOFA_CLASS(SOFA_TEMPLATE(SurfaceSlidingEffector, DataTypes),
             SOFA_TEMPLATE(Effector, DataTypes));

  typedef typename DataTypes::Real Real;
  typedef typename DataTypes::VecCoord VecCoord;
  typedef typename DataTypes::VecDeriv VecDeriv;
  typedef typename DataTypes::Coord Coord;
  typedef typename DataTypes::Deriv Deriv;
  typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

  SurfaceSlidingEffector(MechanicalState *object = nullptr);
  ~SurfaceSlidingEffector() override;

  void init() override;
  // void reinit() override;
  // void reset() override;
  
  void getConstraintViolation(const ConstraintParams *cParams, 
                              sofa::linearalgebra::BaseVector *resV,
                              const sofa::linearalgebra::BaseVector *Jdx) override;

  // void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
  //                              unsigned int& offset) override;

  // Target management for Effector behavior
  sofa::type::vector<Real> getTargetDistance();

protected:
  sofa::Data<sofa::type::vector<Real>> d_targetDistance;

private:
  void updateTargetDistance();
  void computeCurrentDistance();
  
  sofa::type::vector<Real> m_currentDistance;
  
  ////////////////////////// Inherited from Effector ////////////////////////////
  using Effector<DataTypes>::d_limitShiftToTarget;
  using Effector<DataTypes>::d_maxShiftToTarget;
  using Effector<DataTypes>::d_maxSpeed;
  
  ////////////////////////// Inherited from SurfaceSlidingConstraint ////////////
  using softrobots::constraint::SurfaceSlidingConstraint<DataTypes>::d_pointIndex;
  using softrobots::constraint::SurfaceSlidingConstraint<DataTypes>::m_distance;
};

#if !defined(SOFTROBOTS_INVERSE_SURFACESLIDINGEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API
    SurfaceSlidingEffector<sofa::defaulttype::Vec3Types>;
#endif

} // namespace softrobotsinverse::constraint