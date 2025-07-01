#pragma once
#include <sofa/core/visual/VisualParams.h>

#include <SoftRobots.Inverse/component/constraint/SurfaceSlidingSensor.h>

namespace softrobotsinverse::constraint {

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::type::vector;


template <class DataTypes>
SurfaceSlidingSensor<DataTypes>::SurfaceSlidingSensor(MechanicalState *object)
    : Sensor<DataTypes>(object), softrobots::constraint::SurfaceSlidingConstraint<DataTypes>(object) {
    }


template <class DataTypes>
SurfaceSlidingSensor<DataTypes>::~SurfaceSlidingSensor() {}

template <class DataTypes>
void SurfaceSlidingSensor<DataTypes>::getConstraintResolution(
    std::vector<ConstraintResolution *> &resTab, unsigned int &offset) {
//   // Use DoNothingConstraintResolution like SurfacePressureSensor
//   for (unsigned int i = 0; i < d_pointIndex.getValue().size(); i++) {
//     DoNothingConstraintResolution *cr = new DoNothingConstraintResolution();
//     resTab[offset++] = cr; 
//  }
    DoNothingConstraintResolution *cr = new DoNothingConstraintResolution();
    resTab[offset++] = cr;
}

} // namespace softrobotsinverse::constraint