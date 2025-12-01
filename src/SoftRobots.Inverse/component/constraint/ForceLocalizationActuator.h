#pragma once

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{
    using softrobotsinverse::behavior::Actuator;
    using sofa::core::topology::BaseMeshTopology;
    using sofa::core::visual::VisualParams;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::helper::ReadAccessor;
    using sofa::core::ConstVecCoordId;

/**
 * This component is used to solve an inverse problem by estimating forces on a set of candidate points.
 * Unlike ForcePointActuator, which applies the SAME force (variable) to all points,
 * ForceLocalizationActuator solves for an INDEPENDENT force at each point.
 * This effectively allows the solver to localize where the force is applied.
*/
template< class DataTypes >
class ForceLocalizationActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ForceLocalizationActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                                  DataVecCoord;
    typedef sofa::Data<VecDeriv>                                  DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                               DataMatrixDeriv;


public:
    ForceLocalizationActuator(MechanicalState* = nullptr);
    ~ForceLocalizationActuator() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////

    ///////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    ////////////////////////////////////////////////////////////////////////

protected:

    sofa::Data<sofa::type::vector<sofa::Index>>  d_indices;
    sofa::Data<Real>                             d_maxForce;
    sofa::Data<Real>                             d_minForce;
    sofa::Data<Real>                             d_initForce;
    
    // Output: contains the force found for each index. 
    // If direction is fixed: size = indices.size()
    // If direction is free: size = indices.size() * 3
    sofa::Data<sofa::type::vector<Real>>         d_force; 
    
    sofa::Data<Deriv>                            d_direction;
    sofa::Data<Real>                             d_epsilon;

    sofa::Data<bool>                             d_showForce;
    sofa::Data<Real>                             d_visuScale;

    sofa::Size                                   m_dim;

    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::d_constraintIndex ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_lambdaInit ;
    using Actuator<DataTypes>::m_epsilon ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaInit ;
    using Actuator<DataTypes>::m_hasEpsilon ;
    using Actuator<DataTypes>::m_nbLines ;

    void initLimit();
    void initData();
    void updateLimit();

private:

    void setUpData();

};

#if !defined(SOFTROBOTS_INVERSE_FORCELOCALIZATIONACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API ForceLocalizationActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
