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
 * This component applies a force on a surface and allows the solver to "slide" the force
 * to a better location by optimizing barycentric coordinates (u, v).
 * 
 * It constructs a local linearization of the contact problem:
 * - Variable 0: Normal Force Magnitude
 * - Variable 1: Step along U (barycentric)
 * - Variable 2: Step along V (barycentric)
 */
template< class DataTypes >
class SlidingForceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingForceActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename sofa::core::topology::BaseMeshTopology::Triangle Triangle;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                                  DataVecCoord;
    typedef sofa::Data<VecDeriv>                                  DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                               DataMatrixDeriv;


public:
    SlidingForceActuator(MechanicalState* = nullptr);
    ~SlidingForceActuator() override;

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

    // Inputs
    sofa::Data<sofa::type::vector<unsigned int>> d_triangleIndices; // Which triangles are active
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_localCoords; // current (U, V, 0) Cartesian in tangent plane for each point.
    
    sofa::Data<Real>                             d_maxForce;
    sofa::Data<Real>                             d_minForce;
    sofa::Data<sofa::type::Vec3>                 d_initForce;
    sofa::Data<Real>                             d_maxForceStep; // Max change in force magnitude per iteration (0 = no limit)
    
    sofa::Data<Real>                             d_maxStepSize; // Trust region for sliding (Cartesian step limit)
    sofa::Data<Real>                             d_stepDamping; // Damping factor for sliding step (0..1)
    sofa::Data<Real>                             d_epsilonForce; // regularization
    sofa::Data<Real>                             d_epsilonSliding; // regularization for sliding
    sofa::Data<Real>                             d_ridgeForce; // Ridge for force variable
    sofa::Data<Real>                             d_ridgeSliding; // Ridge for sliding variable
    sofa::Data<Real>                             d_jacobianScaleFactor; // Factor to scale constraint Jacobian rows (default 1.0). Use ~3.33 for 30% YM smoothing.
    sofa::Data<Real>                             d_epsilon; // Overall regularization (deprecated, use d_epsilonForce and d_epsilonSliding instead)

    // Internal State
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentForces; // Force from previous step (needed for gradients)
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentLocation; // Current location in world coordinates
    sofa::type::vector<unsigned int>             m_activeTriangles; // Internal copy
    sofa::type::vector<sofa::type::Vec3>         m_activeLocalCoords; 

    // Visualization
    sofa::Data<bool>                             d_showForce;
    sofa::Data<Real>                             d_visuScale;

    sofa::Size                                   m_dim;

    // Each active point contributes 5 constraint rows:
    //   indices [0, 1, 2] = force components (x, y, z)
    //   indices [3, 4]    = sliding tangent step (dU, dV) in triangle frame
    static constexpr unsigned int s_rowsPerPoint = 5;

    // Numerical tolerances; chosen empirically, tune if convergence issues arise.
    static constexpr Real s_squaredEpsilon   = Real(1e-12); // for norm2() / squared-length / area checks
    static constexpr Real s_normEpsilon      = Real(1e-9);  // for norm() / scalar-length checks
    static constexpr Real s_fallbackForceMag = Real(1e-3); // fallback magnitude when initForce is zero

    // Topology link
    sofa::SingleLink<SlidingForceActuator<DataTypes>, sofa::core::topology::BaseMeshTopology, sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> d_topology;

    ////////////////////////// Inherited attributes ////////////////////////////
    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::d_constraintIndex ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_lambdaInit ;
    using Actuator<DataTypes>::m_nbLines ;
    using Actuator<DataTypes>::m_epsilon ;
    using Actuator<DataTypes>::m_hasEpsilon ;
    using Actuator<DataTypes>::m_hasLambdaInit ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    ////////////////////////////////////////////////////////////////////////////

    void initData();
    void updateLimit();
    void projectToMesh(unsigned int& triIdx, sofa::type::Vec3& localCoords);

public:
    Real getEpsilonSliding() const { return d_epsilonSliding.getValue(); }
    bool hasEpsilonSliding() const { return d_epsilonSliding.isSet(); }

    Real getEpsilonForce() const { return d_epsilonForce.getValue(); }
    bool hasEpsilonForce() const { return d_epsilonForce.isSet(); }

    Real getRidgeForce() const { return d_ridgeForce.getValue(); }
    bool hasRidgeForce() const { return d_ridgeForce.isSet(); }

    Real getRidgeSliding() const { return d_ridgeSliding.getValue(); }
    bool hasRidgeSliding() const { return d_ridgeSliding.isSet(); }

};

#if !defined(SOFTROBOTS_INVERSE_SLIDINGFORCEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SlidingForceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace