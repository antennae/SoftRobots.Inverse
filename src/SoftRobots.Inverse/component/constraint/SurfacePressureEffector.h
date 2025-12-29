#pragma once

// #include <SoftRobots/component/constraint/SurfaceSlidingConstraint.h>

#include <SoftRobots.Inverse/component/behavior/Effector.h>
#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace softrobotsinverse::constraint {

using sofa::core::ConstraintParams;
using sofa::core::VecCoordId;
using sofa::core::behavior::ConstraintResolution;
using sofa::helper::ReadAccessor;
using softrobotsinverse::behavior::Effector;

template <class DataTypes>
class SurfacePressureEffector
    : public Effector<DataTypes>,
      public softrobots::constraint::SurfacePressureModel<DataTypes> {
public:
  SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureEffector, DataTypes),
             SOFA_TEMPLATE(Effector, DataTypes));

  typedef typename DataTypes::Real Real;
  typedef typename DataTypes::VecCoord VecCoord;
  typedef typename DataTypes::VecDeriv VecDeriv;
  typedef typename DataTypes::Coord Coord;
  typedef typename DataTypes::Deriv Deriv;
  typedef typename DataTypes::MatrixDeriv MatrixDeriv;
  typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

  SurfacePressureEffector(MechanicalState *object = nullptr);
  ~SurfacePressureEffector() override;

  void init() override;
  // void reinit() override;
  // void reset() override;
  
  void getConstraintViolation(const ConstraintParams *cParams, 
                              sofa::linearalgebra::BaseVector *resV,
                              const sofa::linearalgebra::BaseVector *Jdx) override;

  void buildConstraintMatrix(const ConstraintParams* cParams,
                             sofa::Data<MatrixDeriv> &cMatrix,
                             unsigned int &cIndex,
                             const sofa::Data<sofa::type::vector<typename DataTypes::Coord>> &x) override;

  // virtual void storeResults(sofa::type::vector<double> &delta) override;
  // void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
  //                              unsigned int& offset) override;


protected:
  sofa::Data<Real> d_targetPressure;
  sofa::Data<Real> d_initPressure;
  sofa::Data<Real> d_weight;

private:
  // void updateTargetPressure();
  // void computeCurrentPressure();
  
  sofa::Data<Real> m_targetVolume;
  // sofa::Data<Real> m_initPressure;

  sofa::Data<Real> m_currentPressure;
  
  ////////////////////////// Inherited from Effector ////////////////////////////
  using Effector<DataTypes>::d_limitShiftToTarget;
  using Effector<DataTypes>::d_maxShiftToTarget;
  using Effector<DataTypes>::d_maxSpeed;
  
  ////////////////////////// Inherited from SurfacePressureModel ////////////
  using softrobots::constraint::SurfacePressureModel<DataTypes>::getCavityVolume ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::m_state ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_cavityVolume ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_initialCavityVolume;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxPressure;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minPressure;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqPressure;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_pressure ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_volumeGrowth ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqVolumeGrowth ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_triangles ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_quads ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_flipNormal ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_constraintIndex ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::d_componentState ;
  using softrobots::constraint::SurfacePressureModel<DataTypes>::m_nbLines ;
};

#if !defined(SOFTROBOTS_INVERSE_SURFACEPRESSUREEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API
    SurfacePressureEffector<sofa::defaulttype::Vec3Types>;
#endif

} // namespace softrobotsinverse::constraint