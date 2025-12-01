#pragma once

#include <SoftRobots.Inverse/component/constraint/ForceLocalizationActuator.h>
#include <sofa/core/visual/VisualParams.h>
#include <math.h>

namespace softrobotsinverse::constraint
{
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::type::Mat;
using sofa::type::Vec3;
using sofa::type::vector;
using sofa::linearalgebra::BaseVector;
using sofa::core::VecCoordId ;
using sofa::helper::rabs;

template<class DataTypes>
ForceLocalizationActuator<DataTypes>::ForceLocalizationActuator(MechanicalState* object)
    : Inherit1(object)

    , d_indices(this->initData(&d_indices, "indices",
                       "Indices of the candidate points on the model"))

    , d_maxForce(this->initData(&d_maxForce, "maxForce",
                       "Maximum force allowed per point"))

    , d_minForce(this->initData(&d_minForce, "minForce",
                       "Minimum force allowed per point"))

    , d_initForce(this->initData(&d_initForce, Real(0.0), "initForce",
                       "Initial force guess"))

    , d_force(this->initData(&d_force, "force",
                       "Output: The force found at each index. Size depends on direction."))

    , d_direction(this->initData(&d_direction, "direction",
                           "Direction of the force. If [0,0,0], direction is optimized (3 vars per point)."))

    , d_epsilon(this->initData(&d_epsilon, Real(1e-3), "penalty",
                           "Regularization term. Higher value promotes smaller forces (sparsity)."))

    , d_showForce(this->initData(&d_showForce, false, "showForce",
                           "Visualize the estimated forces"))

    , d_visuScale(this->initData(&d_visuScale, Real(0.1), "visuScale",
                           "Scale of the force arrows"))

{
    setUpData();
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::setUpData()
{
    d_force.setReadOnly(true);
    d_showForce.setGroup("Visualization");
    d_visuScale.setGroup("Visualization");
}


template<class DataTypes>
ForceLocalizationActuator<DataTypes>::~ForceLocalizationActuator()
{
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::init()
{
    Inherit1::init();

    if(m_state==nullptr)
        msg_error(this) << "There is no mechanical state associated with this node. "
                            "the object is deactivated. "
                            "To remove this error message fix your scene possibly by "
                            "adding a MechanicalObject." ;
    initData();
    initLimit();
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::reinit()
{
    initData();
    initLimit();
}

template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::initData()
{
    unsigned int nbIndices = d_indices.getValue().size();
    bool isFreeDirection = (d_direction.getValue().norm() < 1e-10);
    unsigned int varsPerPoint = isFreeDirection ? Deriv::total_size : 1;
    
    m_dim = nbIndices * varsPerPoint;

    if(d_epsilon.isSet())
    {
        m_hasEpsilon = true;
        m_epsilon = d_epsilon.getValue();
    }

    m_lambdaInit.resize(m_dim);
    m_lambdaMax.resize(m_dim);
    m_lambdaMin.resize(m_dim);
    
    // Initialize output vector
    sofa::type::vector<Real> forceVec;
    forceVec.resize(m_dim, d_initForce.getValue());
    d_force.setValue(forceVec);

    if(d_initForce.isSet())
    {
        m_hasLambdaInit = true;
        std::fill(m_lambdaInit.begin(), m_lambdaInit.end(), d_initForce.getValue());
    }
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::initLimit()
{
    if(d_maxForce.isSet()) m_hasLambdaMax = true;
    if(d_minForce.isSet()) m_hasLambdaMin = true;

    updateLimit();
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::updateLimit()
{
    if(d_maxForce.isSet())
        std::fill(m_lambdaMax.begin(), m_lambdaMax.end(), d_maxForce.getValue());

    if(d_minForce.isSet())
        std::fill(m_lambdaMin.begin(), m_lambdaMin.end(), d_minForce.getValue());
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                          DataMatrixDeriv &cMatrix,
                                                          unsigned int &cIndex,
                                                          const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    d_constraintIndex.setValue(cIndex);
    unsigned int startConstraintIndex = cIndex;
    
    const auto& indices = d_indices.getValue();
    unsigned int nbIndices = indices.size();

    Deriv direction = d_direction.getValue();
    bool isFreeDirection = (direction.norm() < 1e-10);

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    if(isFreeDirection) // 3 variables per point
    {
        for(unsigned int i=0; i<nbIndices; i++)
        {
            sofa::Index pointIndex = indices[i];
            if(pointIndex >= m_state->getSize()) continue;

            // Add 3 rows for this point (X, Y, Z)
            for(unsigned int j=0; j<Deriv::total_size; j++)
            {
                Deriv dir;
                dir[j] = 1;
                // Create a new constraint row
                MatrixDerivRowIterator rowIterator = matrix.writeLine(cIndex);
                rowIterator.addCol(pointIndex, dir);
                cIndex++;
            }
        }
    }
    else // 1 variable per point (Fixed direction)
    {
        direction /= direction.norm();
        
        for(unsigned int i=0; i<nbIndices; i++)
        {
            sofa::Index pointIndex = indices[i];
            if(pointIndex >= m_state->getSize()) continue;

            // Create a new constraint row
            MatrixDerivRowIterator rowIterator = matrix.writeLine(cIndex);
            rowIterator.addCol(pointIndex, direction);
            cIndex++;
        }
    }

    cMatrix.endEdit();
    m_nbLines = cIndex - startConstraintIndex;
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                           BaseVector *resV,
                                                           const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(Jdx);

    const auto& startId = sofa::helper::getReadAccessor(d_constraintIndex);
    
    // Target is zero violation (forces are variables, not constraints on motion)
    for(unsigned int i=0; i<m_dim; i++)
        resV->set(startId + i, 0.);
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    SOFA_UNUSED(delta);
    WriteAccessor<sofa::Data<vector<Real>>> force = d_force;

    // Map lambda back to force vector
    // Since we created 1 variable per index (or 3), the mapping is 1:1
    for(unsigned int i=0; i<m_dim; i++)
    {
        if (i < lambda.size())
            force[i] = lambda[i];
    }

    updateLimit();
    Actuator<DataTypes>::storeResults(lambda, delta);
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields() || !d_showForce.getValue())
        return;

    vparams->drawTool()->setLightingEnabled(true);

    ReadAccessor<sofa::Data<sofa::type::vector<sofa::Index>>> indices = sofa::helper::getReadAccessor(d_indices);
    ReadAccessor<sofa::Data<Real>> visuScale = sofa::helper::getReadAccessor(d_visuScale);
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    ReadAccessor<sofa::Data<vector<Real>>> force = d_force;
    Deriv direction = d_direction.getValue();
    bool isFreeDirection = (direction.norm() < 1e-10);

    static const sofa::type::RGBAColor color(0.8, 0.0, 0.0, 1); // Red arrows

    unsigned int varsPerPoint = isFreeDirection ? Deriv::total_size : 1;

    if(!isFreeDirection) 
        direction /= direction.norm();

    for(unsigned int i=0; i < indices.size(); i++)
    {
        sofa::Index pointIdx = indices[i];
        if(pointIdx < m_state->getSize())
        {
            Vec3 position = Vec3(positions[pointIdx][0], positions[pointIdx][1], positions[pointIdx][2]);
            Vec3 forceVec;

            if(isFreeDirection)
            {
                unsigned int base = i * varsPerPoint;
                forceVec = Vec3(force[base], force[base+1], force[base+2]);
            }
            else
            {
                forceVec = Vec3(direction[0], direction[1], direction[2]) * force[i];
            }
            
            Real norm = forceVec.norm();
            if (norm > 1e-6)
            {
                // Draw arrow proportional to force
                vparams->drawTool()->drawArrow(position, position + forceVec * visuScale, 
                                             visuScale * 0.2, color, 4);
            }
        }
    }

    vparams->drawTool()->restoreLastState();
}

} // namespace
