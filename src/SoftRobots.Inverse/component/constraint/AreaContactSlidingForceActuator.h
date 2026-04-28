#pragma once

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SoftRobots.Inverse/component/config.h>

#include <atomic>
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
 * AreaContactSlidingForceActuator:
 * Area contact force actuator with sigmoid pressure distribution.
 * Extends spherical sliding with a contact radius DOF.
 *
 * 6 DOFs per contact: (px, py, pz, dTheta, dPhi, dR)
 *
 * - px, py, pz: pressure (force per unit area) — total force = p * sum(C_t * A_t)
 * - dTheta, dPhi: sliding on S^Par (same as SphericalSlidingForceActuator)
 * - dR: change in contact radius
 *
 * Sigmoid mask: C(d,r) = 1/(1+exp(k*(d-r)))
 *
 * Requires a precomputed .spar file from build_spherical_param.py.
 */
template< class DataTypes >
class AreaContactSlidingForceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(AreaContactSlidingForceActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

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
    AreaContactSlidingForceActuator(MechanicalState* = nullptr);
    ~AreaContactSlidingForceActuator() override;

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

    sofa::Data<std::string>                      d_sparFile;
    sofa::Data<sofa::type::vector<Real>>         d_initTheta;
    sofa::Data<sofa::type::vector<Real>>         d_initPhi;
    sofa::Data<sofa::type::vector<Real>>         d_initRadius;     ///< Initial contact radius per contact

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
    sofa::Data<Real>                             d_ridgeRadius;     ///< Ridge for radius DOF
    sofa::Data<Real>                             d_epsilonRadius;   ///< Regularization for radius constraint
    sofa::Data<Real>                             d_jacobianScaleFactor;
    sofa::Data<Real>                             d_dirMomentum;
    sofa::Data<Real>                             d_slideMomentum;

    // Radius-specific parameters
    sofa::Data<Real>                             d_sigmoidK;        ///< Sigmoid sharpness (1/mm)
    sofa::Data<Real>                             d_cutoffThreshold; ///< C below this is skipped
    sofa::Data<Real>                             d_maxRadiusStep;   ///< Max delta_r per iteration
    sofa::Data<Real>                             d_minRadius;       ///< Minimum allowed radius
    sofa::Data<Real>                             d_maxRadius;       ///< Maximum allowed radius
    sofa::Data<Real>                             d_radiusDamping;   ///< Damping for radius updates
    sofa::Data<Real>                             d_radiusMomentum;  ///< EMA momentum for radius (0=off, ~0.5-0.8=smooth)

    // Outputs
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentForces;
    sofa::Data<sofa::type::vector<sofa::type::Vec3>> d_currentLocation;
    sofa::Data<sofa::type::vector<Real>>              d_currentRadiusOut; ///< Current radius (output)

    // Visualization
    sofa::Data<bool>                             d_showForce;
    sofa::Data<Real>                             d_visuScale;

    // ── S^Par data (loaded from .spar file) ──

    std::vector<sofa::type::Vec3>                m_sparVertices;
    std::vector<sofa::type::Vec3>                m_origVertices;
    std::vector<std::array<unsigned int, 3>>     m_sparTriangles;

    // ── Per-contact state ──

    unsigned int                                 m_nbContacts {0};
    sofa::type::vector<Real>                     m_currentTheta;
    sofa::type::vector<Real>                     m_currentPhi;
    sofa::type::vector<unsigned int>             m_currentTriSpar;
    sofa::type::vector<Real>                     m_currentAlpha;
    sofa::type::vector<Real>                     m_currentBeta;
    sofa::type::vector<sofa::type::Vec3>         m_smoothForces;
    sofa::type::vector<Real>                     m_slideMomentumTheta;
    sofa::type::vector<Real>                     m_slideMomentumPhi;
    sofa::type::vector<Real>                     m_rowNormTheta;
    sofa::type::vector<Real>                     m_rowNormPhi;

    // ── Area contact state ──

    sofa::type::vector<Real>                     m_currentRadius;
    sofa::type::vector<Real>                     m_rowNormR;
    sofa::type::vector<Real>                     m_radiusMomentumVal; ///< EMA accumulator for radius

    /// Cached patch triangle data
    struct PatchTriangle {
        unsigned int triIdx;
        Real weight;      ///< C(d_t, r) * A_t
        Real dCdr;        ///< k * C * (1 - C) * A_t
    };
    std::vector<std::vector<PatchTriangle>>      m_patches;
    std::atomic<bool>                            m_patchDirty {true};
    sofa::type::vector<Real>                     m_lastPatchRadius;  ///< Radius at last patch membership rebuild
    sofa::type::vector<sofa::type::Vec3>         m_lastPatchCenter;  ///< Center at last patch membership rebuild

    // Pre-computed mesh data (updated per step in recomputePatches)
    std::vector<sofa::type::Vec3>                m_triCentroids;
    std::vector<Real>                            m_triAreas;

    sofa::Size                                   m_dim;

    // Each active contact contributes 6 constraint rows:
    //   indices [0, 1, 2] = pressure components (px, py, pz)
    //   indices [3, 4]    = sliding step in spherical coords (dTheta, dPhi)
    //   index   [5]       = radius step (dR)
    static constexpr unsigned int s_rowsPerContact = 6;

    // Numerical tolerances; chosen empirically, tune if convergence issues arise.
    static constexpr Real s_squaredEpsilon   = Real(1e-12); // for norm2() / squared-length / area checks AND division-by-zero guards
    static constexpr Real s_fallbackForceMag = Real(1e-3);  // fallback magnitude when initForce is zero

    sofa::SingleLink<AreaContactSlidingForceActuator<DataTypes>, sofa::core::topology::BaseMeshTopology, sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK> d_topology;

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
    void recomputePatches(const VecCoord& pos);

    static sofa::type::Vec3 sphericalToCart(Real theta, Real phi);

    static bool radialBarycentric(const sofa::type::Vec3& v0,
                                  const sofa::type::Vec3& v1,
                                  const sofa::type::Vec3& v2,
                                  const sofa::type::Vec3& p_sph,
                                  Real& alpha, Real& beta);

    bool findTriangleOnSphere(Real theta, Real phi,
                              unsigned int& triIdx,
                              Real& alpha, Real& beta) const;

    sofa::type::Vec3 sphericalToMesh(unsigned int triIdx, Real alpha, Real beta,
                                     const VecCoord& pos) const;

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

    Real getRidgeRadius() const { return d_ridgeRadius.getValue(); }
    bool hasRidgeRadius() const { return d_ridgeRadius.isSet(); }

    Real getEpsilonRadius() const { return d_epsilonRadius.getValue(); }
    bool hasEpsilonRadius() const { return d_epsilonRadius.isSet(); }
};

#if !defined(SOFTROBOTS_INVERSE_AREACONTACTSLIDINGFORCEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API AreaContactSlidingForceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
