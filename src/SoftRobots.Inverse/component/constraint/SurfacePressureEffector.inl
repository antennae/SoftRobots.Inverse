#pragma once

#include <SoftRobots.Inverse/component/constraint/SurfacePressureEffector.h>
#include <sofa/core/objectmodel/ComponentState.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/type/Vec.h>

namespace softrobotsinverse::constraint {

using sofa::core::objectmodel::ComponentState;
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::core::topology::BaseMeshTopology;

template <class DataTypes>
SurfacePressureEffector<DataTypes>::SurfacePressureEffector(
    MechanicalState *object)
    : Effector<DataTypes>(object),
      softrobots::constraint::SurfacePressureModel<DataTypes>(object),
      d_targetPressure(initData(&d_targetPressure, "targetPressure",
                                "Target pressure for the cavity")),
      d_initPressure(initData(&d_initPressure, (Real)0.0, "initPressure",
                              "Initial pressure in the cavity")),
      d_weight(initData(&d_weight, (Real)1.0, "weight", "Weight of the constraint")) {}

template <class DataTypes>
SurfacePressureEffector<DataTypes>::~SurfacePressureEffector() {}

template <class DataTypes> void SurfacePressureEffector<DataTypes>::init() {
  // Initialize base constraint first
  softrobots::constraint::SurfacePressureModel<DataTypes>::init();
  Effector<DataTypes>::init();

  if (this->d_componentState.getValue() != ComponentState::Valid)
    return;

  // d_pressure.setValue(d_initPressure.getValue()); // d_pressure is read-only in base class



  ReadAccessor<sofa::Data<VecCoord>> positions =
      *m_state->read(sofa::core::vec_id::read_access::position);
  Real volume = getCavityVolume(positions.ref());
  d_initialCavityVolume.setValue(volume);
  d_cavityVolume.setValue(volume);
  m_currentPressure.setValue(d_initPressure.getValue());
}

template <class DataTypes>
void SurfacePressureEffector<DataTypes>::getConstraintViolation(
    const ConstraintParams *cParams, sofa::linearalgebra::BaseVector *resV,
    const sofa::linearalgebra::BaseVector *Jdx) {

  sofa::helper::AdvancedTimer::stepBegin(
      "SurfacePressureEffector::getConstraintViolation");

  const auto &constraintIndex =
      sofa::helper::getReadAccessor(this->d_constraintIndex);

  double v = getCavityVolume(m_state->readPositions().ref());
  d_cavityVolume.setValue(v);
  d_pressure.setValue(
      (d_initPressure.getValue() * d_initialCavityVolume.getValue()) /
      d_cavityVolume.getValue());

  // Update pressure based on Boyle's Law: P * V = P_init * V_init
  // This logic calculates what the target VOLUME should be to achieve d_targetPressure
  // assuming P*V = const. 
  
  // Actually, d_pressure is OUTPUT. 
  // We want to force the volume to be such that P = targetPressure.
  // V_target = (P_init * V_init) / P_target
  
  // Wait, if d_pressure.setValue logic was commented out in init, maybe we don't track current pressure here?
  // But we have d_initPressure.
  
  Real finalTargetVolume = 0;
  if (std::abs(d_targetPressure.getValue()) > 1e-9) {
      finalTargetVolume = (d_initPressure.getValue() * d_initialCavityVolume.getValue()) / d_targetPressure.getValue();
  }

  // Use Effector's getTarget to handle smoothing/limiting
  Real smoothedTargetVolume = this->getTarget(finalTargetVolume, d_cavityVolume.getValue());

  m_targetVolume.setValue(smoothedTargetVolume);

  // Apply weight to the violation
  Real dfree =
      Jdx->element(0) + (d_cavityVolume.getValue() - m_targetVolume.getValue()) * d_weight.getValue();

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





// template <class DataTypes>
// void SurfacePressureEffector<DataTypes>::getConstraintViolation(
//     const ConstraintParams *cParams, sofa::linearalgebra::BaseVector *resV,
//     const sofa::linearalgebra::BaseVector *Jdx) {

//   sofa::helper::AdvancedTimer::stepBegin(
//       "SurfacePressureEffector::getConstraintViolation");

//   const auto &constraintIndex =
//       sofa::helper::getReadAccessor(this->d_constraintIndex);

//   double v = getCavityVolume(m_state->readPositions().ref());
//   d_cavityVolume.setValue(v);

//   if (std::abs(v) > 1e-9) {
//       m_currentPressure = (d_initPressure.getValue() * d_initialCavityVolume.getValue()) / v;
//   }
//   d_pressure.setValue(m_currentPressure.getValue());

//   Real pressureError = m_currentPressure.getValue() - d_targetPressure.getValue();

// //   std::cout<<"Current Pressure: " << currentPressure
// //            << ", Target Pressure: " << d_targetPressure.getValue()
// //            << ", Pressure Error: " << pressureError << std::endl;

//   // Apply weight to the violation
//   std::cout << "Jdx element: " << Jdx->element(0) << std::endl;
//   std::cout<<"Pressure Error: " << pressureError << ", Weight: " << d_weight.getValue() << std::endl;
//   Real dfree = Jdx->element(0) + pressureError * d_weight.getValue();

//   resV->set(constraintIndex, dfree);

//   sofa::helper::AdvancedTimer::stepEnd(
//       "SurfacePressureEffector::getConstraintViolation");
// }

// template<class DataTypes>
// void SurfacePressureEffector<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
//                                                             sofa::Data<MatrixDeriv> &cMatrix,
//                                                             unsigned int &cIndex,
//                                                             const sofa::Data<sofa::type::vector<typename DataTypes::Coord>> &x)
// {
//     if(this->d_componentState.getValue() != ComponentState::Valid)
//             return ;

//     SOFA_UNUSED(cParams);

//     this->d_constraintIndex.setValue(cIndex);
//     const auto& constraintIndex = sofa::helper::getReadAccessor(this->d_constraintIndex);

//     using Quad = typename BaseMeshTopology::Quad;
//     using Triangle = typename BaseMeshTopology::Triangle;
    
//     ReadAccessor<sofa::Data<sofa::type::vector<Quad>>>     quadList = this->d_quads;
//     ReadAccessor<sofa::Data<sofa::type::vector<Triangle>>> triList  = this->d_triangles;

//     using MatrixDerivRowIterator = typename MatrixDeriv::RowIterator;
//     using Deriv = typename DataTypes::Deriv;
//     using VecCoord = typename DataTypes::VecCoord;
//     using Real = typename DataTypes::Real;

//     MatrixDeriv& matrix = *cMatrix.beginEdit();
//     matrix.begin();
//     MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

//     cIndex++;

    
//     // Real P = d_pressure.getValue();


//     // Real V = d_cavityVolume.getValue();
//     // Real P = m_currentPressure.getValue();
//     Real weight = d_weight.getValue();

//     // std::cout<<"Building Constraint Matrix: Pressure P = " << P << ", Volume V = " << V << ", Weight = " << weight << std::endl;

//     // // Jacobian Scale Factor: dP/dV = -P/V
//     // // We apply the weight here too, so the solver sees: weight * J_pressure
//     Real scaleFactor = 0.0;
//     // if (std::abs(V) > 1e-9) {
//     //     scaleFactor = - (P / V) * weight;
//     // }
//     scaleFactor = weight;


//     VecCoord positions = x.getValue();
//     for (const Quad& quad :  quadList)
//     {
//         Deriv triangle1Normal = cross(positions[quad[1]] - positions[quad[0]], positions[quad[3]] - positions[quad[0]])/2.0;
//         Deriv triangle2Normal = cross(positions[quad[3]] - positions[quad[2]], positions[quad[1]] - positions[quad[2]])/2.0;
//         Deriv quadNormal      = triangle1Normal + triangle2Normal;
//         if(this->d_flipNormal.getValue())
//             quadNormal = -quadNormal;

//         quadNormal *= scaleFactor; // Apply weight

//         for (unsigned i=0; i<4; i++)
//         {
//             rowIterator.addCol(quad[i], quadNormal*(1.0/4.0));
//         }
//     }

//     for (const Triangle& triangle : triList)
//     {
//         Deriv triangleNormal = cross(positions[triangle[1]]- positions[triangle[0]], positions[triangle[2]] -positions[triangle[0]])/2.0;
//         if(this->d_flipNormal.getValue())
//             triangleNormal = -triangleNormal;
        
//         triangleNormal *= scaleFactor; // Apply weight

//         for (unsigned i=0; i<3; i++)
//         {
//             rowIterator.addCol(triangle[i], triangleNormal*(1.0/3.0));
//         }
//     }

//     cMatrix.endEdit();
//     this->m_nbLines = cIndex - constraintIndex;
// }


// template<class DataTypes>
// void SurfacePressureEffector<DataTypes>::storeResults(sofa::type::vector<double> &delta)
// {
//     // d_volumeGrowth.setValue(delta[0]);
//     for(auto val : delta)
//     {
//         std::cout << "Delta value: " << val << std::endl;
//     }
// }

} // namespace softrobotsinverse::constraint
