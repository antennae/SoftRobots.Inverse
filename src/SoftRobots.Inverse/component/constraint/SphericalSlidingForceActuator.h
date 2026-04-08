#pragma once

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SoftRobots.Inverse/component/config.h>

#include <string>
#include <vector>

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
 * SphericalSlidingForceActuator:
 * Applies a force that slides across a mesh using spherical parameterization (S^Par).
 * Uses 2 global spherical DOFs (dTheta, dPhi) instead of per-triangle barycentric (dwB, dwC).
 * This eliminates Jacobian discontinuities at triangle boundaries.
 *
 * DOFs per contact: 5 = (Fx, Fy, Fz, dTheta, dPhi)
 *
 * Requires a precomputed .spar file from build_spherical_param.py.
 */
template< class DataTypes >
class SphericalSlidingForceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SphericalSlidingForceActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

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
    SphericalSlidingForceActuator(MechanicalState* = nullptr);
    ~SphericalSlidingForceActuator() override;

    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;

    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;

    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;

protected:
    // ── Scene-facing data fields ──

    sofa::Data<std::string>                      d_sparFile;      ///< Path to .spar binary file
    sofa::Data<sofa::type::vector<Real>>         d_initTheta;     ///< Initial theta per contact
    sofa::Data<sofa::type::vector<Real>>         d_initPhi;       ///< Initial phi per contact

    sofa::Data<Real>                             d_maxForce;
    sofa::Data<Real>                             d_minForce;
    sofa::Data<sofa::type::Vec3>                 d_initForce;
    sofa::Data<Real>                             d_maxForceStep;

    sofa::Data<Real>                             d_maxStepSize;     ///< Max (dTheta, dPhi) step in radians
    sofa::Data<Real>                             d_stepDamping;
    sofa::Data<Real>                             d_epsilonForce;
    sofa::Data<Real>                             d_epsilonSliding;
    sofa::Data<Real>                             d_ridgeForce;
    sofa::Data<Real>                             d_ridgeSliding;
    sofa::Data<Real>                             d_jacobianScaleFactor;
    sofa::Data<Real>                             d_dirMomentum;     ///< EMA momentum for force direction
    sofa::Data<Real>                             d_slideMomentum;   ///< EMA momentum for QP sliding output
    sofa::Data<Real>                             d_annealRate;      ///< Annealing rate: damping ramps from d_stepDamping to 1.0 over steps. 0=off.
    sofa::Data<unsigned int>                     d_stagnationWindow;///< Steps of small sliding before perturbation kick. 0=off.
    sofa::Data<Real>                             d_perturbRadius;   ///< Perturbation magnitude (radians) when stagnation detected

    // Outputs
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentForces;
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentLocation;

    // Visualization
    sofa::Data<bool>                             d_showForce;
    sofa::Data<Real>                             d_visuScale;

    // ── S^Par data (loaded from .spar file) ──

    std::vector<sofa::type::Vec3>                m_sparVertices;    ///< (V,3) vertices on unit sphere
    std::vector<sofa::type::Vec3>                m_origVertices;    ///< (V,3) original mesh vertices (rest shape)
    std::vector<std::array<unsigned int, 3>>     m_sparTriangles;   ///< (F,3) face connectivity

    // ── Per-contact state ──

    unsigned int                                 m_nbContacts {0};
    sofa::type::vector<Real>                     m_currentTheta;
    sofa::type::vector<Real>                     m_currentPhi;
    sofa::type::vector<unsigned int>             m_currentTriSpar;   ///< Current triangle on S^Par
    sofa::type::vector<Real>                     m_currentAlpha;     ///< Barycentric alpha in S^Par triangle
    sofa::type::vector<Real>                     m_currentBeta;
    sofa::type::vector<sofa::type::Vec3>         m_smoothForces;
    sofa::type::vector<Real>                     m_slideMomentumTheta;
    sofa::type::vector<Real>                     m_slideMomentumPhi;
    sofa::type::vector<Real>                     m_rowNormTheta;     ///< Jacobian row norm for dTheta (for normalization)
    sofa::type::vector<Real>                     m_rowNormPhi;       ///< Jacobian row norm for dPhi
    unsigned int                                 m_stepCount {0};    ///< Step counter for annealing
    sofa::type::vector<unsigned int>             m_stagnationCount;  ///< Per-contact stagnation counter

    sofa::Size                                   m_dim;

    sofa::SingleLink<SphericalSlidingForceActuator<DataTypes>, sofa::core::topology::BaseMeshTopology, sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> d_topology;

    using Actuator<DataTypes>::m_state;
    using Actuator<DataTypes>::d_constraintIndex;
    using Actuator<DataTypes>::m_lambdaMax;
    using Actuator<DataTypes>::m_lambdaMin;
    using Actuator<DataTypes>::m_lambdaInit;
    using Actuator<DataTypes>::m_hasLambdaMax;
    using Actuator<DataTypes>::m_hasLambdaMin;
    using Actuator<DataTypes>::m_hasLambdaInit;
    using Actuator<DataTypes>::m_nbLines;

    // ── Internal methods ──

    bool loadSparFile(const std::string& path);
    void initData();
    void updateLimit();

    /// Convert (theta, phi) to unit sphere point
    static sofa::type::Vec3 sphericalToCart(Real theta, Real phi);

    /// Radial barycentric: ray from origin through p_sph → triangle (v0,v1,v2)
    static bool radialBarycentric(const sofa::type::Vec3& v0,
                                  const sofa::type::Vec3& v1,
                                  const sofa::type::Vec3& v2,
                                  const sofa::type::Vec3& p_sph,
                                  Real& alpha, Real& beta);

    /// Find containing triangle on S^Par for a given sphere point
    bool findTriangleOnSphere(Real theta, Real phi,
                              unsigned int& triIdx,
                              Real& alpha, Real& beta) const;

    /// Map S^Par triangle + barycentric to mesh position using DEFORMED vertices
    sofa::type::Vec3 sphericalToMesh(unsigned int triIdx, Real alpha, Real beta,
                                     const VecCoord& pos) const;

    /// Compute sliding Jacobians: d(alpha,beta)/d(theta,phi)
    void computeSlidingJacobian(unsigned int triIdx, Real theta, Real phi,
                                Real& da_dtheta, Real& db_dtheta,
                                Real& da_dphi, Real& db_dphi) const;

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

#if !defined(SOFTROBOTS_INVERSE_SPHERICALSLIDINGFORCEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SphericalSlidingForceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
