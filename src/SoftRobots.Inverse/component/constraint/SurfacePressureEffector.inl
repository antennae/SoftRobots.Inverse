#pragma once

#include <SoftRobots.Inverse/component/constraint/SurfacePressureEffector.h>
#include <sofa/core/objectmodel/ComponentState.h>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/type/Vec.h>

namespace softrobotsinverse::constraint {

using sofa::core::objectmodel::ComponentState;
using sofa::core::topology::BaseMeshTopology;
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
                              "Initial pressure in the cavity")),
      d_additionalInitalVolume(initData(&d_additionalInitalVolume, (Real)0.0, "additionalInitialVolume",
                              "Additional initial volume to add to the cavity volume at the beginning of the simulation")),
      d_weight(initData(&d_weight, (Real)1.0, "weight", "Weight of the constraint")),
      d_systemCompliance(initData(&d_systemCompliance, (Real)0.0, "systemCompliance", "Compliance of the system (tubes, sensor) in mm^3/Pa. Relax PV=C if > 0")) {}

template <class DataTypes>
SurfacePressureEffector<DataTypes>::~SurfacePressureEffector() {}

template <class DataTypes> void SurfacePressureEffector<DataTypes>::init() {
  // Initialize base constraint first
  softrobots::constraint::SurfacePressureModel<DataTypes>::init();
  Effector<DataTypes>::init();

  if (this->d_componentState.getValue() != ComponentState::Valid)
    return;

  ReadAccessor<sofa::Data<VecCoord>> positions =
      *m_state->read(sofa::core::vec_id::read_access::position);
  Real meshVolume = getCavityVolume(positions.ref());
  Real extraVolume = d_additionalInitalVolume.getValue();
  Real totalInitialVolume = meshVolume + extraVolume;

  d_initialCavityVolume.setValue(meshVolume);
  d_cavityVolume.setValue(totalInitialVolume);
  
  Real initialPressure = d_initPressure.getValue();
  d_currentPressure.setValue(initialPressure);
  d_pressure.setValue(initialPressure);

  // Thermodynamic Target: P_init * V_gas_init = P_target * V_gas_target
  // V_gas = V_mesh + V_extra + S * (P - P_init)
  Real targetTotalVolume = totalInitialVolume;
  Real S = d_systemCompliance.getValue();
  if (std::abs(d_targetPressure.getValue()) > 1e-9) {
      Real targetGasVolume = (initialPressure * totalInitialVolume) / d_targetPressure.getValue();
      targetTotalVolume = targetGasVolume - S * (d_targetPressure.getValue() - initialPressure);
  }
  d_targetVolume.setValue(targetTotalVolume);

  d_initialCavityVolume.setDisplayed(true);
  d_currentPressure.setDisplayed(true);
  d_pressure.setDisplayed(true);
}

template <class DataTypes>
void SurfacePressureEffector<DataTypes>::getConstraintViolation(
    const ConstraintParams *cParams, sofa::linearalgebra::BaseVector *resV,
    const sofa::linearalgebra::BaseVector *Jdx) {

  sofa::helper::AdvancedTimer::stepBegin(
      "SurfacePressureEffector::getConstraintViolation");

  const auto &constraintIndex =
      sofa::helper::getReadAccessor(this->d_constraintIndex);

  Real currentMeshVolume = getCavityVolume(m_state->readPositions().ref());
  Real extraVolume = d_additionalInitalVolume.getValue();
  Real currentTotalVolume = currentMeshVolume + extraVolume;
  
  d_cavityVolume.setValue(currentTotalVolume);

  Real initialPressure = d_initPressure.getValue();
  Real totalInitialVolume = d_initialCavityVolume.getValue() + extraVolume;
  Real S = d_systemCompliance.getValue();
  
  // Update current pressure based on gas law with system compliance
  // P * (V_mesh + V_extra + S*(P - P_init)) = P_init * V_total_init
  // S*P^2 + (V_curr - S*P_init)*P - C = 0
  if (S > 1e-12) {
      Real V_curr = currentTotalVolume;
      Real C = initialPressure * totalInitialVolume;
      Real b = V_curr - S * initialPressure;
      Real p_current = (-b + std::sqrt(b * b + 4 * S * C)) / (2 * S);
      d_pressure.setValue(p_current);
  } else {
      d_pressure.setValue((initialPressure * totalInitialVolume) / currentTotalVolume);
  }

  // Update target total volume if the user changed the target pressure
  Real targetTotalVolume = currentTotalVolume;
  if (std::abs(d_targetPressure.getValue()) > 1e-9) {
      Real targetGasVolume = (initialPressure * totalInitialVolume) / d_targetPressure.getValue();
      targetTotalVolume = targetGasVolume - S * (d_targetPressure.getValue() - initialPressure);
  }
  d_targetVolume.setValue(targetTotalVolume);

  // The violation is the difference between current total volume and target total volume.
  Real dfree =
      Jdx->element(0) + (currentTotalVolume - targetTotalVolume) * d_weight.getValue();

  resV->set(constraintIndex, dfree);

  sofa::helper::AdvancedTimer::stepEnd(
      "SurfacePressureEffector::getConstraintViolation");
}

template<class DataTypes>
void SurfacePressureEffector<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                            sofa::Data<MatrixDeriv> &cMatrix,
                                                            unsigned int &cIndex,
                                                            const sofa::Data<sofa::type::vector<typename DataTypes::Coord>> &x)
{
    if(this->d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    this->d_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(this->d_constraintIndex);

    using Quad = typename BaseMeshTopology::Quad;
    using Triangle = typename BaseMeshTopology::Triangle;
    
    ReadAccessor<sofa::Data<sofa::type::vector<Quad>>>     quadList = this->d_quads;
    ReadAccessor<sofa::Data<sofa::type::vector<Triangle>>> triList  = this->d_triangles;

    using MatrixDerivRowIterator = typename MatrixDeriv::RowIterator;
    using Deriv = typename DataTypes::Deriv;
    using VecCoord = typename DataTypes::VecCoord;
    using Real = typename DataTypes::Real;

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    matrix.begin();
    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

    cIndex++;

    Real weight = d_weight.getValue();

    VecCoord positions = x.getValue();
    for (const Quad& quad :  quadList)
    {
        Deriv triangle1Normal = cross(positions[quad[1]] - positions[quad[0]], positions[quad[3]] - positions[quad[0]])/2.0;
        Deriv triangle2Normal = cross(positions[quad[3]] - positions[quad[2]], positions[quad[1]] - positions[quad[2]])/2.0;
        Deriv quadNormal      = triangle1Normal + triangle2Normal;
        if(this->d_flipNormal.getValue())
            quadNormal = -quadNormal;

        quadNormal *= weight; // Apply weight

        for (unsigned i=0; i<4; i++)
        {
            rowIterator.addCol(quad[i], quadNormal*(1.0/4.0));
        }
    }

    for (const Triangle& triangle : triList)
    {
        Deriv triangleNormal = cross(positions[triangle[1]]- positions[triangle[0]], positions[triangle[2]] -positions[triangle[0]])/2.0;
        if(this->d_flipNormal.getValue())
            triangleNormal = -triangleNormal;
        
        triangleNormal *= weight; // Apply weight

        for (unsigned i=0; i<3; i++)
        {
            rowIterator.addCol(triangle[i], triangleNormal*(1.0/3.0));
        }
    }

    cMatrix.endEdit();
    this->m_nbLines = cIndex - constraintIndex;
}

template<class DataTypes>
void SurfacePressureEffector<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                  sofa::core::MultiVecDerivId res,
                                                  const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;
    
    Real extraVolume = d_additionalInitalVolume.getValue();
    Real initialPressure = d_initPressure.getValue();
    Real totalInitialVolume = d_initialCavityVolume.getValue() + extraVolume;
    Real S = d_systemCompliance.getValue();
    Real currentTotalVolume = d_cavityVolume.getValue();

    if (S > 1e-12) {
        Real V_curr = currentTotalVolume;
        Real C = initialPressure * totalInitialVolume;
        Real b = V_curr - S * initialPressure;
        // P * (V_curr + S*(P - P_init)) = C
        // S*P^2 + (V_curr - S*P_init)*P - C = 0
        Real p_current = (-b + std::sqrt(b * b + 4 * S * C)) / (2 * S);
        d_pressure.setValue(p_current);
    } else {
        d_pressure.setValue((initialPressure * totalInitialVolume) / currentTotalVolume);
    }

    // Compute actual cavity volume and volume growth from updated positions of mechanical
    // Eulalie.C: For now the position of the mechanical state is not up to date when storeLambda() is called
    //            so the value of delta is one step behind...
    //ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    //d_cavityVolume.setValue(getCavityVolume(positions.ref()));
    d_volumeGrowth.setValue(d_cavityVolume.getValue()-d_initialCavityVolume.getValue());
}

} // namespace softrobotsinverse::constraint