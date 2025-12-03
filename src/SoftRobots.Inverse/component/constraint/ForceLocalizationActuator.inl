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

    , d_sparsity(this->initData(&d_sparsity, Real(0.0), "sparsity",
                           "L1 Regularization term to promote sparsity (selection). Adds a linear cost to the force magnitude. \n"
                           "Ideally used with unilateral constraints (minForce >= 0)."))

    , d_ridge(this->initData(&d_ridge, Real(0.0), "ridge",
                           "L2 Ridge Regularization term for stability (Elastic Net). Adds a quadratic cost to the force magnitude."))

    , d_showForce(this->initData(&d_showForce, false, "showForce",
                           "Visualize the estimated forces"))

    , d_visuScale(this->initData(&d_visuScale, Real(0.1), "visuScale",
                           "Scale of the force arrows"))

{
    setUpData();
}

template<class DataTypes>
double ForceLocalizationActuator<DataTypes>::getSparsity() const
{
    return d_sparsity.getValue();
}

template<class DataTypes>
double ForceLocalizationActuator<DataTypes>::getRidge() const
{
    return d_ridge.getValue();
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
    // If sparsity is active, or if minForce is negative and maxForce is positive, we need 2 variables per component.
    // Otherwise, 1 variable per component (min/max force will define bounds).
    bool useSplitVariables = (d_sparsity.getValue() > 0.0) || (d_minForce.getValue() < 0.0 && d_maxForce.getValue() > 0.0);

    unsigned int varsPerPointComponent = 1;
    if (useSplitVariables) {
        varsPerPointComponent = 2; // For positive and negative parts (u and v)
    }
    
    // Total variables = nbIndices * (3 if free direction, 1 if fixed direction) * (2 if split, 1 if not split)
    unsigned int componentsPerPoint = isFreeDirection ? Deriv::total_size : 1;
    m_dim = nbIndices * componentsPerPoint * varsPerPointComponent;

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
    forceVec.resize(nbIndices * componentsPerPoint, d_initForce.getValue()); // d_force stores the net force, not split variables
    d_force.setValue(forceVec);

    if(d_initForce.isSet())
    {
        m_hasLambdaInit = true;
        // If using split variables, initial guess for u and v is often 0
        if (useSplitVariables) {
            std::fill(m_lambdaInit.begin(), m_lambdaInit.end(), Real(0.0));
        } else {
            std::fill(m_lambdaInit.begin(), m_lambdaInit.end(), d_initForce.getValue());
        }
    }
    
    // Normalize direction if fixed
    if (!isFreeDirection && d_direction.getValue().norm() > 1e-10) {
        Deriv direction = d_direction.getValue();
        direction /= direction.norm();
        d_direction.setValue(direction);
    }
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::initLimit()
{
    bool useSplitVariables = (d_sparsity.getValue() > 0.0) || (d_minForce.getValue() < 0.0 && d_maxForce.getValue() > 0.0);

    if (useSplitVariables) {
        // We strictly require bounds [0, inf] for split variables
        m_hasLambdaMax = true;
        m_hasLambdaMin = true;
    } else {
        if(d_maxForce.isSet()) m_hasLambdaMax = true;
        if(d_minForce.isSet()) m_hasLambdaMin = true;
    }

    updateLimit();
}


template<class DataTypes>
void ForceLocalizationActuator<DataTypes>::updateLimit()
{
    bool isFreeDirection = (d_direction.getValue().norm() < 1e-10);
    bool useSplitVariables = (d_sparsity.getValue() > 0.0) || (d_minForce.getValue() < 0.0 && d_maxForce.getValue() > 0.0);
    unsigned int componentsPerPoint = isFreeDirection ? Deriv::total_size : 1;

    if (useSplitVariables)
    {
        // For split variables (u, v), they must both be non-negative.
        // maxForce and minForce apply to the *net* force (u-v).
        // For individual u, v: [0, infinity] is the standard.
        std::fill(m_lambdaMin.begin(), m_lambdaMin.end(), Real(0.0));
        std::fill(m_lambdaMax.begin(), m_lambdaMax.end(), Real(1e99)); // effectively infinity
        
        // If d_maxForce is set, apply it to the positive part.
        // If d_minForce is set, apply it to the negative part (after negation).
        unsigned int nbIndices = d_indices.getValue().size();
        for (unsigned int i = 0; i < nbIndices; ++i) {
            for (unsigned int comp = 0; comp < componentsPerPoint; ++comp) {
                unsigned int base_idx = (i * componentsPerPoint + comp) * 2; // For u_i_comp, v_i_comp

                if (d_maxForce.isSet()) {
                    m_lambdaMax[base_idx] = d_maxForce.getValue(); // Max for u_i_comp
                }
                if (d_minForce.isSet()) {
                    // For v_i_comp, max bound should be -minForce (since v corresponds to -force)
                    m_lambdaMax[base_idx + 1] = -d_minForce.getValue();
                }
            }
        }
    }
    else // Not using split variables, minForce and maxForce apply directly.
    {
        if(d_maxForce.isSet())
            std::fill(m_lambdaMax.begin(), m_lambdaMax.end(), d_maxForce.getValue());
        else
            std::fill(m_lambdaMax.begin(), m_lambdaMax.end(), Real(1e99)); // effectively infinity

        if(d_minForce.isSet())
            std::fill(m_lambdaMin.begin(), m_lambdaMin.end(), d_minForce.getValue());
        else
            std::fill(m_lambdaMin.begin(), m_lambdaMin.end(), Real(-1e99)); // effectively -infinity
    }
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
    bool useSplitVariables = (d_sparsity.getValue() > 0.0) || (d_minForce.getValue() < 0.0 && d_maxForce.getValue() > 0.0);

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    unsigned int componentsPerPoint = isFreeDirection ? Deriv::total_size : 1;

    for(unsigned int i=0; i<nbIndices; i++)
    {
        sofa::Index pointIndex = indices[i];
        if(pointIndex >= m_state->getSize()) continue;

        if(isFreeDirection) // 3 variables (components) per point
        {
            for(unsigned int j=0; j<componentsPerPoint; j++) // For each (X, Y, Z) component
            {
                Deriv dir;
                dir[j] = 1; // Base direction vector (e.g., [1,0,0], [0,1,0], [0,0,1])

                if (useSplitVariables) {
                    // Add entry for positive component (u)
                    MatrixDerivRowIterator rowIterator_u = matrix.writeLine(cIndex);
                    rowIterator_u.addCol(pointIndex, dir);
                    cIndex++;

                    // Add entry for negative component (v)
                    MatrixDerivRowIterator rowIterator_v = matrix.writeLine(cIndex);
                    rowIterator_v.addCol(pointIndex, -dir); // Negative direction
                    cIndex++;
                } else {
                    // Add single entry for direct force
                    MatrixDerivRowIterator rowIterator = matrix.writeLine(cIndex);
                    rowIterator.addCol(pointIndex, dir);
                    cIndex++;
                }
            }
        }
        else // 1 variable (component) per point (Fixed direction)
        {
            Deriv normalizedDirection = direction;
            if (normalizedDirection.norm() > 1e-10) {
                normalizedDirection /= normalizedDirection.norm();
            } else {
                // Should not happen if initData normalized it, but for safety
                // TODO:
                // normalizedDirection = Deriv(0.0);  
            }

            if (useSplitVariables) {
                // Add entry for positive component (u)
                MatrixDerivRowIterator rowIterator_u = matrix.writeLine(cIndex);
                rowIterator_u.addCol(pointIndex, normalizedDirection);
                cIndex++;

                // Add entry for negative component (v)
                MatrixDerivRowIterator rowIterator_v = matrix.writeLine(cIndex);
                rowIterator_v.addCol(pointIndex, -normalizedDirection); // Negative direction
                cIndex++;
            } else {
                // Add single entry for direct force
                MatrixDerivRowIterator rowIterator = matrix.writeLine(cIndex);
                rowIterator.addCol(pointIndex, normalizedDirection);
                cIndex++;
            }
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

    bool isFreeDirection = (d_direction.getValue().norm() < 1e-10);
    bool useSplitVariables = (d_sparsity.getValue() > 0.0) || (d_minForce.getValue() < 0.0 && d_maxForce.getValue() > 0.0);
    unsigned int componentsPerPoint = isFreeDirection ? Deriv::total_size : 1;

    if (useSplitVariables)
    {
        unsigned int nbIndices = d_indices.getValue().size();
        unsigned int current_lambda_idx = 0;
        for (unsigned int i = 0; i < nbIndices; ++i)
        {
            for (unsigned int comp = 0; comp < componentsPerPoint; ++comp)
            {
                if (current_lambda_idx + 1 < lambda.size()) {
                    Real u_val = lambda[current_lambda_idx];
                    Real v_val = lambda[current_lambda_idx + 1];
                    force[i * componentsPerPoint + comp] = u_val - v_val; // Net force
                } else {
                    force[i * componentsPerPoint + comp] = Real(0.0); // Safety, should not happen
                }
                current_lambda_idx += 2; // Move to next pair (u, v)
            }
        }
    }
    else
    {
        // Original logic: 1:1 mapping
        for(unsigned int i=0; i<m_dim; i++)
        {
            if (i < lambda.size())
                force[i] = lambda[i];
        }
    }
    
    // The force vector (d_force) contains the reconstructed net forces.
    // m_lambdaMin/Max/Init were for the raw lambda values (u,v or single variable).
    // The base Actuator storeResults call can remain as it uses the raw lambda values.

    updateLimit(); // Re-evaluate limits, which might affect lambdaMax/Min for the next step.
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
    ReadAccessor<sofa::Data<vector<Real>>> forceData = d_force; // Use the d_force data directly

    Deriv direction = d_direction.getValue();
    bool isFreeDirection = (direction.norm() < 1e-10);

    static const sofa::type::RGBAColor color(0.8, 0.0, 0.0, 1); // Red arrows

    unsigned int componentsPerPoint = isFreeDirection ? Deriv::total_size : 1;

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
                unsigned int base = i * componentsPerPoint;
                forceVec = Vec3(forceData[base], forceData[base+1], forceData[base+2]);
            }
            else
            {
                forceVec = Vec3(direction[0], direction[1], direction[2]) * forceData[i * componentsPerPoint];
            }
            
            Real norm = forceVec.norm();
            if (norm > 1e-6)
            {
                // Draw arrow proportional to force
                vparams->drawTool()->drawArrow(position, position + forceVec * visuScale, 
                                             std::max(visuScale *0.2, 0.2 ), color, 4);
            }
        }
    }

    vparams->drawTool()->restoreLastState();
}

} // namespace
