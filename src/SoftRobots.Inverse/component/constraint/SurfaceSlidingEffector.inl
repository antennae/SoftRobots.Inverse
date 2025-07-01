#pragma once

#include <SoftRobots.Inverse/component/constraint/SurfaceSlidingEffector.h>
#include <sofa/core/objectmodel/ComponentState.h>

namespace softrobotsinverse::constraint {

using sofa::core::objectmodel::ComponentState;
using sofa::helper::WriteAccessor;
using sofa::helper::ReadAccessor;

template <class DataTypes>
SurfaceSlidingEffector<DataTypes>::SurfaceSlidingEffector(MechanicalState *object)
    : Effector<DataTypes>(object)
    , softrobots::constraint::SurfaceSlidingConstraint<DataTypes>(object)
    , d_targetDistance(initData(&d_targetDistance, "targetDistance", 
                               "Target sliding distances for each point"))
{
}

template <class DataTypes>
SurfaceSlidingEffector<DataTypes>::~SurfaceSlidingEffector()
{
}

template <class DataTypes>
void SurfaceSlidingEffector<DataTypes>::init()
{
    // Initialize base constraint first
    softrobots::constraint::SurfaceSlidingConstraint<DataTypes>::init();
    Effector<DataTypes>::init();
    
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return;
    
    // Initialize target distances with current distances if not set
    if (d_targetDistance.getValue().empty())
    {
        const auto& pointIndices = d_pointIndex.getValue();
        sofa::type::vector<Real> initialTargets(pointIndices.size(), 0.0);
        
        // Get current distances from the constraint's m_distance
        ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances = m_distance;
        if (currentDistances.size() >= pointIndices.size())
        {
            for (size_t i = 0; i < pointIndices.size(); i++)
            {
                initialTargets[i] = currentDistances[i];
            }
        }
        
        d_targetDistance.setValue(initialTargets);
    }
}

template <class DataTypes>
void SurfaceSlidingEffector<DataTypes>::getConstraintViolation(
    const ConstraintParams *cParams,
    sofa::linearalgebra::BaseVector *resV,
    const sofa::linearalgebra::BaseVector *Jdx)
{
    // Update targets with motion limiting if enabled
    updateTargetDistance();
    
    // Get current distances from the base constraint
    ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances = m_distance;
    const auto& targets = getTargetDistance();
    
    // Calculate constraint violation as difference between current and target
    for (size_t i = 0; i < d_pointIndex.getValue().size(); i++)
    {
        Real currentDistance = (i < currentDistances.size()) ? currentDistances[i] : 0.0;
        Real targetDistance = (i < targets.size()) ? targets[i] : 0.0;
        Real violation = currentDistance - targetDistance;
        
        // Add velocity term if provided
        if (Jdx != nullptr)
        {
            violation += Jdx->element(this->d_constraintIndex.getValue() + i);
        }
        
        resV->set(this->d_constraintIndex.getValue() + i, violation);
    }
}

template <class DataTypes>
void SurfaceSlidingEffector<DataTypes>::updateTargetDistance()
{
    if (!d_limitShiftToTarget.getValue())
        return;
        
    const auto& currentTargets = d_targetDistance.getValue();
    sofa::type::vector<Real> newTargets = currentTargets;
    Real maxShift = d_maxShiftToTarget.getValue();
    
    // Get current distances from the constraint
    ReadAccessor<sofa::Data<sofa::type::vector<Real>>> currentDistances = m_distance;
    
    for (size_t i = 0; i < newTargets.size() && i < currentDistances.size(); i++)
    {
        Real currentDistance = currentDistances[i];
        Real difference = newTargets[i] - currentDistance;
        
        // Limit the shift to prevent large jumps
        if (std::abs(difference) > maxShift)
        {
            if (difference > 0)
                newTargets[i] = currentDistance + maxShift;
            else
                newTargets[i] = currentDistance - maxShift;
        }
    }
    
    d_targetDistance.setValue(newTargets);
}

template <class DataTypes>
sofa::type::vector<typename DataTypes::Real> SurfaceSlidingEffector<DataTypes>::getTargetDistance()
{
    return d_targetDistance.getValue();
}

} // namespace softrobotsinverse::constraint