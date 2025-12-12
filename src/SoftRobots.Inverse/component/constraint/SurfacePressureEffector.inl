#pragma once

#include <SoftRobots.Inverse/component/constraint/SurfacePressureEffector.h>
#include <sofa/core/objectmodel/ComponentState.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
namespace softrobotsinverse::constraint {

using sofa::core::objectmodel::ComponentState;
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;

template <class DataTypes>
SurfacePressureEffector<DataTypes>::SurfacePressureEffector(
    MechanicalState *object)
    : Effector<DataTypes>(object),
      softrobots::constraint::SurfacePressureModel<DataTypes>(object),
      d_targetPressure(initData(&d_targetPressure, "targetPressure",
                                "Target pressure for the cavity")),
      d_initPressure(initData(&d_initPressure, (Real)0.0, "initPressure",
                              "Initial pressure in the cavity")) {}

template <class DataTypes>
SurfacePressureEffector<DataTypes>::~SurfacePressureEffector() {}

template <class DataTypes> void SurfacePressureEffector<DataTypes>::init() {
  // Initialize base constraint first
  softrobots::constraint::SurfacePressureModel<DataTypes>::init();
  Effector<DataTypes>::init();

  if (this->d_componentState.getValue() != ComponentState::Valid)
    return;

  d_pressure.setValue(d_initPressure.getValue());

  // Initialize target distances with current distances if not set
  //   if (!d_targetPressure.isSet()) {
  // const auto &pointIndices = d_pointIndex.getValue();
  // sofa::type::vector<Real> initialTargets(pointIndices.size(), 0.0);

  // // Get current distances from the constraint's m_distance
  // ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances =
  // m_distance; if (currentDistances.size() >= pointIndices.size())
  // {
  //     for (size_t i = 0; i < pointIndices.size(); i++)
  //     {
  //         initialTargets[i] = currentDistances[i];
  //     }
  // }

  // d_targetPressure.setValue(0);
  // d_cavityVolume.setValue(getCavityVolume(m_state->readPositions().ref()));
  // m_targetVolume.setValue(d_cavityVolume.getValue());
  //   }

  ReadAccessor<sofa::Data<VecCoord>> positions =
      *m_state->read(sofa::core::vec_id::read_access::position);
  Real volume = getCavityVolume(positions.ref());
  d_initialCavityVolume.setValue(volume);
  d_cavityVolume.setValue(volume);
}

template <class DataTypes>
void SurfacePressureEffector<DataTypes>::getConstraintViolation(
    const ConstraintParams *cParams, sofa::linearalgebra::BaseVector *resV,
    const sofa::linearalgebra::BaseVector *Jdx) {

  sofa::helper::AdvancedTimer::stepBegin(
      "SurfacePressureEffector::getConstraintViolation");
  // Update targets with motion limiting if enabled
  //   updateTargetDistance();

  //   // Get current distances from the base constraint
  //   ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances =
  //       m_distance;
  //   const auto &targets = getTargetDistance();

  const auto &constraintIndex =
      sofa::helper::getReadAccessor(this->d_constraintIndex);

  //   // Use local index counter like PositionEffector does
  //   unsigned int index = 0;
  //   for (size_t i = 0; i < d_pointIndex.getValue().size(); i++) {
  //     Real currentDistance =
  //         (i < currentDistances.size()) ? currentDistances[i] : 0.0;
  //     Real targetDistance = (i < targets.size()) ? targets[i] : 0.0;
  //     Real violation = currentDistance - targetDistance;

  //     // Add velocity term if provided - use local index
  //     if (Jdx != nullptr) {
  //       violation += Jdx->element(index);
  //     }

  //     resV->set(constraintIndex + index, violation);
  //     index++; // Increment local index
  //   }

  double v = getCavityVolume(m_state->readPositions().ref());
  d_cavityVolume.setValue(v);

  // Update pressure based on Boyle's Law: P * V = P_init * V_init
  if (v > 1e-9) {
    d_pressure.setValue(d_initPressure.getValue() *
                        d_initialCavityVolume.getValue() / v);
  }

//   std::cout << "Cavity Volume: " << d_cavityVolume.getValue() << std::endl;
//   std::cout << "current pressure: " << d_pressure.getValue() << std::endl;

//   std::cout << "Target Pressure: " << d_targetPressure.getValue() << std::endl;
//   std::cout << "Target Volume: " << m_targetVolume.getValue() << std::endl;

  m_targetVolume.setValue((d_pressure.getValue() * d_cavityVolume.getValue()) /
                          d_targetPressure.getValue());

  Real dfree =
      Jdx->element(0) + d_cavityVolume.getValue() - m_targetVolume.getValue();

  resV->set(constraintIndex, dfree);

  sofa::helper::AdvancedTimer::stepEnd(
      "SurfacePressureEffector::getConstraintViolation");
}

// template <class DataTypes>
// void SurfacePressureEffector<DataTypes>::updateTargetDistance() {
//   sofa::helper::AdvancedTimer::stepBegin(
//       "SurfacePressureEffector::updateTargetDistance");
//   if (!d_limitShiftToTarget.getValue())
//     return;

//   const auto &currentTargets = d_targetDistance.getValue();
//   sofa::type::vector<Real> newTargets = currentTargets;
//   Real maxShift = d_maxShiftToTarget.getValue();

//   // Get current distances from the constraint
//   ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances =
//       m_distance;

//   for (size_t i = 0; i < newTargets.size() && i < currentDistances.size();
//        i++) {
//     Real currentDistance = currentDistances[i];
//     Real difference = newTargets[i] - currentDistance;

//     // Limit the shift to prevent large jumps
//     if (std::abs(difference) > maxShift) {
//       if (difference > 0)
//         newTargets[i] = currentDistance + maxShift;
//       else
//         newTargets[i] = currentDistance - maxShift;
//     }
//   }

//   d_intermediateTargetDistance.setValue(newTargets);
//   sofa::helper::AdvancedTimer::stepEnd(
//       "SurfacePressureEffector::updateTargetDistance");
// }

// template <class DataTypes>
// sofa::type::vector<typename DataTypes::Real>
// SurfacePressureEffector<DataTypes>::getTargetDistance() {
//   return d_intermediateTargetDistance.getValue();
// }

} // namespace softrobotsinverse::constraint