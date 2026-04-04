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
 * SmoothSlidingForceActuator: 
 * Applies a force that can slide across a mesh using SMOOTH vertex normals.
 * This version uses 2 sliding variables (dU, dV) in a tangent plane that varies 
 * continuously across triangles, preventing chattering at edges.
 */
template< class DataTypes >
class SmoothSlidingForceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SmoothSlidingForceActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

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
    SmoothSlidingForceActuator(MechanicalState* = nullptr);
    ~SmoothSlidingForceActuator() override;

    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;

    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;

    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;

    void getBarycentricCoords(const sofa::type::Vec3& A,
                                         const sofa::type::Vec3& B,
                                         const sofa::type::Vec3& C,
                                         const sofa::type::Vec3& P,
                                         Real& wB, Real& wC);   

protected:
    // Data Inputs
    sofa::Data<sofa::type::vector<unsigned int>> d_triangleIndices; 
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_localCoords; // (U, V, 0)
    
    sofa::Data<Real>                             d_maxForce;
    sofa::Data<Real>                             d_minForce;
    sofa::Data<sofa::type::Vec3>                 d_initForce;
    sofa::Data<Real>                             d_maxForceStep; 
    
    sofa::Data<Real>                             d_maxStepSize; 
    sofa::Data<Real>                             d_stepDamping; 
    sofa::Data<Real>                             d_epsilonForce; 
    sofa::Data<Real>                             d_epsilonSliding; 
    sofa::Data<Real>                             d_ridgeForce; 
    sofa::Data<Real>                             d_ridgeSliding; 
    sofa::Data<Real>                             d_jacobianScaleFactor;
    sofa::Data<Real>                             d_dirMomentum; ///< EMA momentum for sliding Jacobian direction (0=off, ~0.7=smooth)
    sofa::Data<Real>                             d_slideMomentum; ///< EMA momentum for QP sliding output (0=off, ~0.5-0.8=smooth). Filters noisy slide signals so only consistent directions accumulate.

    // Internal State
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentForces;
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentLocation;
    sofa::type::vector<unsigned int>             m_activeTriangles;
    sofa::type::vector<sofa::type::Vec3>         m_activeLocalCoords;
    sofa::type::vector<sofa::type::Vec3>         m_vertexNormals;
    sofa::type::vector<sofa::type::Vec3>         m_smoothForces; ///< EMA-filtered force directions used for Jacobian
    sofa::type::vector<Real>                     m_slideMomentumB; ///< Per-point EMA state for dwB
    sofa::type::vector<Real>                     m_slideMomentumC; ///< Per-point EMA state for dwC
    Real                                         m_meanEdgeLength {1.0}; ///< cached mean edge length (mm) for step-size conversion

    sofa::Data<bool>                             d_showForce;
    sofa::Data<Real>                             d_visuScale;

    sofa::Size                                   m_dim; 

    sofa::SingleLink<SmoothSlidingForceActuator<DataTypes>, sofa::core::topology::BaseMeshTopology, sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> d_topology;

    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::d_constraintIndex ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_lambdaInit ;
    using Actuator<DataTypes>::m_hasLambdaInit ;
    using Actuator<DataTypes>::m_nbLines ;

    void initData();
    void updateLimit();
    void updateVertexNormals();
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

#if !defined(SOFTROBOTS_INVERSE_SMOOTHSLIDINGFORCEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SmoothSlidingForceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
