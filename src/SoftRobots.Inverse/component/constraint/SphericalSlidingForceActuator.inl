#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <limits>

#include <SoftRobots.Inverse/component/constraint/SphericalSlidingForceActuator.h>
#include <sofa/core/visual/VisualParams.h>

namespace softrobotsinverse::constraint
{
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::type::Vec3;
using sofa::core::objectmodel::Data;
using sofa::type::vector;
using sofa::linearalgebra::BaseVector;

// ── Constructor / Destructor ──────────────────────────────────────

template<class DataTypes>
SphericalSlidingForceActuator<DataTypes>::SphericalSlidingForceActuator(MechanicalState* object)
    : Inherit1(object)
    , d_sparFile(initData(&d_sparFile, std::string(""), "sparFile",
                          "Path to .spar binary file (from build_spherical_param.py)"))
    , d_initTheta(initData(&d_initTheta, "initTheta",
                           "Initial theta (polar angle) per contact point"))
    , d_initPhi(initData(&d_initPhi, "initPhi",
                         "Initial phi (azimuthal angle) per contact point"))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max force magnitude"))
    , d_minForce(initData(&d_minForce, "minForce", "Min force magnitude"))
    , d_initForce(initData(&d_initForce, Vec3(0.0, 0.0, 0.0), "initForce",
                           "Initial force guess"))
    , d_maxForceStep(initData(&d_maxForceStep, Real(0.0), "maxForceStep",
                              "Max change in force magnitude per iteration (0 = no limit)"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.05), "maxStepSize",
                             "Max (dTheta, dPhi) step per iteration in radians"))
    , d_stepDamping(initData(&d_stepDamping, Real(0.5), "stepDamping",
                             "Damping factor for sliding step"))
    , d_epsilonForce(initData(&d_epsilonForce, Real(1e-3), "epsilonForce",
                              "Regularization for force constraint"))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding",
                                "Regularization for sliding constraint"))
    , d_ridgeForce(initData(&d_ridgeForce, Real(1e-12), "ridgeForce",
                            "Ridge for force variable"))
    , d_ridgeSliding(initData(&d_ridgeSliding, Real(1e-12), "ridgeSliding",
                              "Ridge for sliding variable"))
    , d_jacobianScaleFactor(initData(&d_jacobianScaleFactor, Real(1.0),
                                     "jacobianScaleFactor",
                                     "Factor to scale constraint Jacobian rows"))
    , d_dirMomentum(initData(&d_dirMomentum, Real(0.0), "dirMomentum",
                             "EMA momentum for force direction (0=off, ~0.7=smooth)"))
    , d_slideMomentum(initData(&d_slideMomentum, Real(0.0), "slideMomentum",
                               "EMA momentum for QP sliding output (0=off, ~0.5-0.8=smooth)"))
    , d_annealRate(initData(&d_annealRate, Real(0.0), "annealRate",
                            "Damping annealing rate: 0=off, >0 ramps damping from stepDamping to 1.0 with time constant annealRate steps"))
    , d_stagnationWindow(initData(&d_stagnationWindow, (unsigned int)(0), "stagnationWindow",
                                  "Steps of small sliding before perturbation kick (0=off)"))
    , d_perturbRadius(initData(&d_perturbRadius, Real(0.3), "perturbRadius",
                               "Perturbation magnitude in radians when stagnation detected"))
    , d_currentForces(initData(&d_currentForces, "currentForces",
                               "Current forces applied"))
    , d_currentLocation(initData(&d_currentLocation, "currentLocation",
                                 "Current force locations in world coordinates"))
    , d_showForce(initData(&d_showForce, false, "showForce", "Visualize forces"))
    , d_visuScale(initData(&d_visuScale, Real(0.1), "visuScale",
                           "Scale for visualization"))
    , d_topology(initLink("topology", "Mesh topology"))
{
    this->d_showForce.setGroup("Visualization");
    this->d_visuScale.setGroup("Visualization");
}

template<class DataTypes>
SphericalSlidingForceActuator<DataTypes>::~SphericalSlidingForceActuator()
{
}

// ── .spar file loader ─────────────────────────────────────────────

template<class DataTypes>
bool SphericalSlidingForceActuator<DataTypes>::loadSparFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        msg_error() << "Cannot open .spar file: " << path;
        return false;
    }

    // Read magic "SPAR"
    char magic[4];
    file.read(magic, 4);
    if (magic[0] != 'S' || magic[1] != 'P' || magic[2] != 'A' || magic[3] != 'R') {
        msg_error() << "Invalid .spar file (bad magic): " << path;
        return false;
    }

    // Read header
    uint32_t version, nVerts, nFaces;
    file.read(reinterpret_cast<char*>(&version), 4);
    file.read(reinterpret_cast<char*>(&nVerts), 4);
    file.read(reinterpret_cast<char*>(&nFaces), 4);

    if (version != 1) {
        msg_error() << "Unsupported .spar version: " << version;
        return false;
    }

    msg_info() << "Loading S^Par: " << nVerts << " vertices, " << nFaces << " faces";

    // Read vertices_spar: nVerts x 3 x float64
    m_sparVertices.resize(nVerts);
    for (uint32_t i = 0; i < nVerts; ++i) {
        double xyz[3];
        file.read(reinterpret_cast<char*>(xyz), 3 * sizeof(double));
        m_sparVertices[i] = Vec3(Real(xyz[0]), Real(xyz[1]), Real(xyz[2]));
    }

    // Read theta_phi: nVerts x 2 x float64 (skip — we recompute from initTheta/initPhi)
    std::streamoff const offset = std::streamoff{nVerts} * 2 * std::streamoff{sizeof(double)};
    file.seekg(offset, std::ios::cur);

    // Read faces: nFaces x 3 x uint32
    m_sparTriangles.resize(nFaces);
    for (uint32_t i = 0; i < nFaces; ++i) {
        uint32_t idx[3];
        file.read(reinterpret_cast<char*>(idx), 3 * sizeof(uint32_t));
        m_sparTriangles[i] = {idx[0], idx[1], idx[2]};
    }

    // Read vertices_original: nVerts x 3 x float64
    m_origVertices.resize(nVerts);
    for (uint32_t i = 0; i < nVerts; ++i) {
        double xyz[3];
        file.read(reinterpret_cast<char*>(xyz), 3 * sizeof(double));
        m_origVertices[i] = Vec3(Real(xyz[0]), Real(xyz[1]), Real(xyz[2]));
    }

    if (!file.good()) {
        msg_error() << "Error reading .spar file: " << path;
        return false;
    }

    msg_info() << "S^Par loaded successfully";
    return true;
}

// ── Spherical ↔ Cartesian ─────────────────────────────────────────

template<class DataTypes>
sofa::type::Vec3 SphericalSlidingForceActuator<DataTypes>::sphericalToCart(Real theta, Real phi)
{
    Real const st = std::sin(theta);
    return Vec3(st * std::cos(phi), st * std::sin(phi), std::cos(theta));
}

// ── Radial barycentric (ray from origin) ──────────────────────────

template<class DataTypes>
bool SphericalSlidingForceActuator<DataTypes>::radialBarycentric(
    const Vec3& v0, const Vec3& v1, const Vec3& v2,
    const Vec3& p_sph, Real& alpha, Real& beta)
{
    // M = (v1 - v0) x (v2 - v0) — triangle normal (not normalized)
    Vec3 const M = sofa::type::cross(v1 - v0, v2 - v0);
    Real const M_dot_p = M * p_sph;
    if (std::abs(M_dot_p) < s_squaredEpsilon) {
        alpha = beta = 0;
        return false;  // ray parallel to triangle
    }
    Real const t = (M * v0) / M_dot_p;
    if (t < 0) {
        alpha = beta = 0;
        return false;  // triangle behind origin
    }

    // Intersection point on triangle plane
    Vec3 const p_plane = p_sph * t;
    Real const M_sq = M * M;

    // alpha, beta via Eq. 7
    alpha = (M * sofa::type::cross(p_plane - v0, v2 - v0)) / M_sq;
    beta  = (M * sofa::type::cross(v1 - v0, p_plane - v0)) / M_sq;
    return true;
}

// ── Find triangle on S^Par ────────────────────────────────────────

template<class DataTypes>
bool SphericalSlidingForceActuator<DataTypes>::findTriangleOnSphere(
    Real theta, Real phi,
    unsigned int& triIdx, Real& alpha, Real& beta) const
{
    Vec3 const p_sph = sphericalToCart(theta, phi);

    // Linear scan — sufficient for ~8K triangles at init + once per storeResults
    Real bestDist = std::numeric_limits<Real>::max();
    unsigned int bestTri = m_sparTriangles.size();  // sentinel: any value >= size means "not found"
    Real bestAlpha = 0, bestBeta = 0;

    for (unsigned int fi = 0; fi < m_sparTriangles.size(); ++fi) {
        const auto& f = m_sparTriangles[fi];
        Real a, b;
        if (!radialBarycentric(m_sparVertices[f[0]], m_sparVertices[f[1]],
                               m_sparVertices[f[2]], p_sph, a, b))
            continue;

        Real const gamma = Real(1.0) - a - b;
        // FP slack on barycentric containment: a/b/gamma can be slightly negative
        // due to round-off when the ray hits exactly on an edge.
        if (a >= Real(-1e-8) && b >= Real(-1e-8) && gamma >= Real(-1e-8)) {
            triIdx = fi;
            alpha = a;
            beta = b;
            return true;
        }

        // Track closest for fallback
        Real const ac = std::max(Real(0), a);
        Real const bc = std::max(Real(0), std::min(b, Real(1) - ac));
        Vec3 proj = m_sparVertices[f[0]]
                  + ac * (m_sparVertices[f[1]] - m_sparVertices[f[0]])
                  + bc * (m_sparVertices[f[2]] - m_sparVertices[f[0]]);
        Real const pnorm = proj.norm();
        if (pnorm > s_squaredEpsilon) proj /= pnorm;
        Real const dist = (proj - p_sph).norm();
        if (dist < bestDist) {
            bestDist = dist;
            bestTri = fi;
            bestAlpha = ac;
            bestBeta = bc;
        }
    }

    if (bestTri < m_sparTriangles.size()) {
        triIdx = bestTri;
        alpha = bestAlpha;
        beta = bestBeta;
        return true;
    }
    return false;
}

// ── Map S^Par → mesh position (using DEFORMED vertices) ───────────

template<class DataTypes>
sofa::type::Vec3 SphericalSlidingForceActuator<DataTypes>::sphericalToMesh(
    unsigned int triIdx, Real alpha, Real beta, const VecCoord& pos) const
{
    const auto& f = m_sparTriangles[triIdx];
    // Use deformed vertex positions from the mechanical state
    Vec3 const v0(pos[f[0]]);
    Vec3 const v1(pos[f[1]]);
    Vec3 const v2(pos[f[2]]);
    return v0 + alpha * (v1 - v0) + beta * (v2 - v0);
}

// ── Sliding Jacobian (Eq. 21) ─────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::computeSlidingJacobian(
    unsigned int triIdx, Real theta, Real phi,
    Real& da_dtheta, Real& db_dtheta,
    Real& da_dphi, Real& db_dphi) const
{
    const auto& f = m_sparTriangles[triIdx];
    const Vec3& v0 = m_sparVertices[f[0]];
    const Vec3& v1 = m_sparVertices[f[1]];
    const Vec3& v2 = m_sparVertices[f[2]];

    Vec3 const M = sofa::type::cross(v1 - v0, v2 - v0);
    Real const M_sq = M * M;
    if (M_sq < s_squaredEpsilon) {
        da_dtheta = db_dtheta = da_dphi = db_dphi = 0;
        return;
    }
    Vec3 const M_over_Msq = M / M_sq;

    // Derivatives of P_sph w.r.t. theta, phi
    Real ct = std::cos(theta), st = std::sin(theta);
    Real cp = std::cos(phi),   sp = std::sin(phi);
    Vec3 const dP_dtheta(ct * cp, ct * sp, -st);
    Vec3 const dP_dphi(-st * sp, st * cp, Real(0));

    Vec3 const e1 = v1 - v0;  // edge 0→1
    Vec3 const e2 = v2 - v0;  // edge 0→2

    da_dtheta = M_over_Msq * sofa::type::cross(dP_dtheta, e2);
    db_dtheta = M_over_Msq * sofa::type::cross(e1, dP_dtheta);
    da_dphi   = M_over_Msq * sofa::type::cross(dP_dphi, e2);
    db_dphi   = M_over_Msq * sofa::type::cross(e1, dP_dphi);
}

// ── Init / Reinit ─────────────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::init()
{
    Inherit1::init();
    if (!this->d_topology.get())
        this->d_topology.set(this->getContext()->getMeshTopology());
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::reinit()
{
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::initData()
{
    // Load S^Par
    const std::string& sparPath = d_sparFile.getValue();
    if (sparPath.empty() || !loadSparFile(sparPath)) {
        msg_error() << "sparFile is required. Generate with build_spherical_param.py";
        return;
    }

    // Determine number of contact points from initTheta
    const auto& initTheta = d_initTheta.getValue();
    const auto& initPhi = d_initPhi.getValue();
    m_nbContacts = initTheta.size();
    if (m_nbContacts == 0) {
        msg_error() << "initTheta is empty — at least one contact point required";
        return;
    }
    if (initPhi.size() != m_nbContacts) {
        msg_error() << "initTheta and initPhi must have the same size";
        return;
    }

    this->m_dim = m_nbContacts * s_rowsPerPoint;
    this->m_nbLines = this->m_dim;

    // Initialize per-contact state
    m_currentTheta.resize(m_nbContacts);
    m_currentPhi.resize(m_nbContacts);
    m_currentTriSpar.resize(m_nbContacts);
    m_currentAlpha.resize(m_nbContacts);
    m_currentBeta.resize(m_nbContacts);

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        m_currentTheta[i] = initTheta[i];
        m_currentPhi[i] = initPhi[i];
        if (!findTriangleOnSphere(m_currentTheta[i], m_currentPhi[i],
                                  m_currentTriSpar[i],
                                  m_currentAlpha[i], m_currentBeta[i])) {
            msg_warning() << "Contact " << i << ": could not find triangle for "
                          << "(theta=" << m_currentTheta[i]
                          << ", phi=" << m_currentPhi[i] << ")";
        }
    }

    // Lambda bounds
    this->m_lambdaInit.assign(this->m_dim, 0.0);
    this->m_lambdaMax.assign(this->m_dim, std::numeric_limits<Real>::max());
    this->m_lambdaMin.assign(this->m_dim, std::numeric_limits<Real>::lowest());

    // Initialize forces from face normal (same pattern as SmoothSlidingForceActuator)
    sofa::type::vector<Vec3> currentForces;
    currentForces.resize(m_nbContacts);
    if (this->d_topology.get() && this->m_state) {
        ReadAccessor<Data<VecCoord>> const pos = this->m_state->readPositions();
        Vec3 const f0 = this->d_initForce.getValue();
        Real fMag = f0.norm();
        if (fMag == 0.0) fMag = s_fallbackForceMag;
        for (unsigned int i = 0; i < m_nbContacts; ++i) {
            unsigned int const triIdx = m_currentTriSpar[i];
            if (triIdx < m_sparTriangles.size()) {
                const auto& f = m_sparTriangles[triIdx];
                // Use deformed mesh positions for face normal
                if (f[0] < pos.size() && f[1] < pos.size() && f[2] < pos.size()) {
                    Vec3 n = sofa::type::cross(
                        Vec3(pos[f[1]]) - Vec3(pos[f[0]]),
                        Vec3(pos[f[2]]) - Vec3(pos[f[0]]));
                    n.normalize();
                    currentForces[i] = n * fMag;
                }
            }
        }
    }
    this->d_currentForces.setValue(currentForces);

    // Initialize smooth force directions
    m_smoothForces = currentForces;

    // Initialize sliding momentum accumulators
    m_slideMomentumTheta.assign(m_nbContacts, Real(0));
    m_slideMomentumPhi.assign(m_nbContacts, Real(0));
    m_stagnationCount.assign(m_nbContacts, 0);
    m_stepCount = 0;

    // Compute initial row norms from initial (theta, phi)
    m_rowNormTheta.assign(m_nbContacts, Real(1));
    m_rowNormPhi.assign(m_nbContacts, Real(1));
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Real da_dt, db_dt, da_dp, db_dp;
        computeSlidingJacobian(m_currentTriSpar[i], m_currentTheta[i], m_currentPhi[i],
                               da_dt, db_dt, da_dp, db_dp);
        Real const rnT = std::sqrt((da_dt + db_dt) * (da_dt + db_dt) + da_dt * da_dt + db_dt * db_dt);
        Real const rnP = std::sqrt((da_dp + db_dp) * (da_dp + db_dp) + da_dp * da_dp + db_dp * db_dp);
        m_rowNormTheta[i] = (rnT > s_squaredEpsilon) ? rnT : Real(1);
        m_rowNormPhi[i]   = (rnP > s_squaredEpsilon) ? rnP : Real(1);
    }

    // Warm-start lambda
    this->m_hasLambdaInit = true;
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        this->m_lambdaInit[i*s_rowsPerPoint +0] = currentForces[i][0];
        this->m_lambdaInit[i*s_rowsPerPoint +1] = currentForces[i][1];
        this->m_lambdaInit[i*s_rowsPerPoint +2] = currentForces[i][2];
        this->m_lambdaInit[i*s_rowsPerPoint +3] = 0.0;  // dTheta
        this->m_lambdaInit[i*s_rowsPerPoint +4] = 0.0;  // dPhi
    }

    // Compute initial world-space locations
    if (this->m_state) {
        ReadAccessor<Data<VecCoord>> const pos = this->m_state->readPositions();
        sofa::type::vector<Vec3> currentLocations;
        currentLocations.resize(m_nbContacts);
        for (unsigned int i = 0; i < m_nbContacts; ++i) {
            currentLocations[i] = sphericalToMesh(
                m_currentTriSpar[i], m_currentAlpha[i], m_currentBeta[i],
                pos.ref());
        }
        this->d_currentLocation.setValue(currentLocations);
    }
}

// ── Update bounds ─────────────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::updateLimit()
{
    Real const maxF = this->d_maxForce.isSet() ? this->d_maxForce.getValue() : std::numeric_limits<Real>::max();
    Real const minF = this->d_minForce.isSet() ? this->d_minForce.getValue() : std::numeric_limits<Real>::lowest();
    Real const maxStep = this->d_maxStepSize.getValue();  // already in radians
    Real const maxForceStep = this->d_maxForceStep.getValue();
    const auto& currentForces = this->d_currentForces.getValue();

    this->m_hasLambdaMax = true;
    this->m_hasLambdaMin = true;

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Vec3 curF = (i < currentForces.size()) ? currentForces[i] : Vec3(0, 0, 0);

        if (maxForceStep > 0.0) {
            this->m_lambdaMin[i*s_rowsPerPoint +0] = std::max(minF, curF[0] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerPoint +0] = std::min(maxF, curF[0] + maxForceStep);
            this->m_lambdaMin[i*s_rowsPerPoint +1] = std::max(minF, curF[1] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerPoint +1] = std::min(maxF, curF[1] + maxForceStep);
            this->m_lambdaMin[i*s_rowsPerPoint +2] = std::max(minF, curF[2] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerPoint +2] = std::min(maxF, curF[2] + maxForceStep);
        } else {
            this->m_lambdaMin[i*s_rowsPerPoint +0] = minF; this->m_lambdaMax[i*s_rowsPerPoint +0] = maxF;
            this->m_lambdaMin[i*s_rowsPerPoint +1] = minF; this->m_lambdaMax[i*s_rowsPerPoint +1] = maxF;
            this->m_lambdaMin[i*s_rowsPerPoint +2] = minF; this->m_lambdaMax[i*s_rowsPerPoint +2] = maxF;
        }
        // Sliding bounds: scaled for normalized rows.
        // Physical dTheta = lambda_norm * factor / rowNorm.
        // To bound physical step to maxStep radians: lambda_norm ≤ maxStep * rowNorm / factor.
        Real const factor = this->d_jacobianScaleFactor.getValue();
        Real const rnT = (i < m_rowNormTheta.size()) ? m_rowNormTheta[i] : Real(1);
        Real const rnP = (i < m_rowNormPhi.size())   ? m_rowNormPhi[i]   : Real(1);
        Real const boundT = maxStep * rnT / ((factor > s_squaredEpsilon) ? factor : Real(1));
        Real const boundP = maxStep * rnP / ((factor > s_squaredEpsilon) ? factor : Real(1));
        this->m_lambdaMin[i*s_rowsPerPoint +3] = -boundT; this->m_lambdaMax[i*s_rowsPerPoint +3] = boundT;
        this->m_lambdaMin[i*s_rowsPerPoint +4] = -boundP; this->m_lambdaMax[i*s_rowsPerPoint +4] = boundP;
    }
}

// ── buildConstraintMatrix ─────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::buildConstraintMatrix(
    const ConstraintParams* cParams,
    DataMatrixDeriv &cMatrix,
    unsigned int &cIndex,
    const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    this->d_constraintIndex.setValue(cIndex);
    unsigned int const startId = cIndex;
    if (!this->d_topology.get() || m_sparTriangles.empty()) return;

    const VecCoord& pos = x.getValue();
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    Real const factor = this->d_jacobianScaleFactor.getValue();

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        unsigned int const triSpar = m_currentTriSpar[i];
        if (triSpar >= m_sparTriangles.size()) continue;
        const auto& f = m_sparTriangles[triSpar];

        Real const alpha = m_currentAlpha[i];
        Real const beta  = m_currentBeta[i];
        Real const wA = Real(1.0) - alpha - beta;
        Real const wB = alpha;
        Real const wC = beta;

        // ── Rows 0,1,2: Force (Fx, Fy, Fz) ──
        // Barycentric interpolation on DEFORMED mesh triangle
        for (int dim = 0; dim < 3; ++dim) {
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            row.addCol(f[0], factor * Deriv(dim==0?wA:0, dim==1?wA:0, dim==2?wA:0));
            row.addCol(f[1], factor * Deriv(dim==0?wB:0, dim==1?wB:0, dim==2?wB:0));
            row.addCol(f[2], factor * Deriv(dim==0?wC:0, dim==1?wC:0, dim==2?wC:0));
        }

        // ── Rows 3,4: Sliding (dTheta, dPhi) ──
        // Compute sliding Jacobians on S^Par
        Real da_dtheta, db_dtheta, da_dphi, db_dphi;
        computeSlidingJacobian(triSpar, m_currentTheta[i], m_currentPhi[i],
                               da_dtheta, db_dtheta, da_dphi, db_dphi);

        // Get smooth force direction for coupling
        Vec3 smoothF = (m_smoothForces.size() > i)
                     ? m_smoothForces[i] : Vec3(0, 0, 0);
        Real const smoothFMag = smoothF.norm();
        if (smoothFMag < Real(1e-6)) {
            // Fall back to face normal on deformed mesh
            Vec3 const nFace = sofa::type::cross(
                Vec3(pos[f[1]]) - Vec3(pos[f[0]]),
                Vec3(pos[f[2]]) - Vec3(pos[f[0]]));
            Real const a2 = nFace.norm();
            smoothF = (a2 > s_squaredEpsilon) ? nFace / a2 : Vec3(0, 0, 1);
        } else {
            smoothF /= smoothFMag;
        }

        // Row 3: dTheta — normalized so row magnitude ≈ factor (like force rows).
        // Without normalization, row magnitude ≈ factor * rowNorm ≈ factor * 20,
        // which is 34x larger than force rows, causing the QP to always saturate
        // the sliding bounds. Normalization makes the QP treat sliding and force
        // on equal footing (same as SmoothSlidingForceActuator's natural balance).
        // Lambda bounds in updateLimit are scaled by rowNorm/factor to preserve
        // the physical maxStepSize in radians.
        {
            Real const rn = std::sqrt(
                (da_dtheta + db_dtheta) * (da_dtheta + db_dtheta)
                + da_dtheta * da_dtheta + db_dtheta * db_dtheta);
            m_rowNormTheta[i] = (rn > s_squaredEpsilon) ? rn : Real(1);
            Real const invNorm = Real(1.0) / m_rowNormTheta[i];
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            row.addCol(f[0], factor * smoothF * (-(da_dtheta + db_dtheta) * invNorm));
            row.addCol(f[1], factor * smoothF * (da_dtheta * invNorm));
            row.addCol(f[2], factor * smoothF * (db_dtheta * invNorm));
        }

        // Row 4: dPhi — same normalization
        {
            Real const rn = std::sqrt(
                (da_dphi + db_dphi) * (da_dphi + db_dphi)
                + da_dphi * da_dphi + db_dphi * db_dphi);
            m_rowNormPhi[i] = (rn > s_squaredEpsilon) ? rn : Real(1);
            Real const invNorm = Real(1.0) / m_rowNormPhi[i];
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            row.addCol(f[0], factor * smoothF * (-(da_dphi + db_dphi) * invNorm));
            row.addCol(f[1], factor * smoothF * (da_dphi * invNorm));
            row.addCol(f[2], factor * smoothF * (db_dphi * invNorm));
        }
    }

    cMatrix.endEdit();
    this->m_nbLines = cIndex - startId;
}

// ── getConstraintViolation ────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::getConstraintViolation(
    const ConstraintParams* cParams, BaseVector *resV, const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams); SOFA_UNUSED(Jdx);
    unsigned int const totalDim = m_nbContacts * s_rowsPerPoint;
    const auto& constraintId = sofa::helper::getReadAccessor(this->d_constraintIndex);
    for (unsigned int i = 0; i < totalDim; ++i)
        resV->set(constraintId + i, 0.);
}

// ── storeResults ──────────────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::storeResults(
    vector<double> &lambda, vector<double> &delta)
{
    if (!this->d_topology.get() || !this->m_state || m_sparTriangles.empty()) return;
    ReadAccessor<Data<VecCoord>> const pos = this->m_state->readPositions();
    WriteAccessor<Data<sofa::type::vector<Vec3>>> currentForces = this->d_currentForces;
    Real const baseDamping = this->d_stepDamping.getValue();
    Real const maxStep = this->d_maxStepSize.getValue();
    Real const annealRate = this->d_annealRate.getValue();
    unsigned int const stagnationWindow = this->d_stagnationWindow.getValue();
    Real const perturbRadius = this->d_perturbRadius.getValue();

    ++m_stepCount;

    // [Claude 2026-04-06] Annealing: ramp damping from baseDamping toward 1.0
    // damping(t) = 1.0 - (1.0 - baseDamping) * exp(-t / annealRate)
    Real damping = baseDamping;
    if (annealRate > Real(0)) {
        damping = Real(1.0) - (Real(1.0) - baseDamping) * std::exp(-Real(m_stepCount) / annealRate);
    }

    static unsigned int s_storeCount = 0;
    ++s_storeCount;

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Real const factor = this->d_jacobianScaleFactor.getValue();
        Real const Fx = lambda[i*s_rowsPerPoint +0] * factor;
        Real const Fy = lambda[i*s_rowsPerPoint +1] * factor;
        Real const Fz = lambda[i*s_rowsPerPoint +2] * factor;
        // Denormalize sliding: QP lambda is in normalized space.
        // Physical dTheta = lambda_norm * factor / rowNorm.
        Real const rnT = (i < m_rowNormTheta.size() && m_rowNormTheta[i] > s_squaredEpsilon) ? m_rowNormTheta[i] : Real(1);
        Real const rnP = (i < m_rowNormPhi.size()   && m_rowNormPhi[i]   > s_squaredEpsilon) ? m_rowNormPhi[i]   : Real(1);
        Real dTheta = lambda[i*s_rowsPerPoint +3] * factor / rnT;
        Real dPhi   = lambda[i*s_rowsPerPoint +4] * factor / rnP;

        // ── Diagnostic print (every 10 steps) ──
        if (s_storeCount % 10 == 1) {
            Real da_dt, db_dt, da_dp, db_dp;
            computeSlidingJacobian(m_currentTriSpar[i], m_currentTheta[i], m_currentPhi[i],
                                   da_dt, db_dt, da_dp, db_dp);
            Vec3 const sF = (m_smoothForces.size() > i) ? m_smoothForces[i] : Vec3(0,0,0);
            Real const sFMag = sF.norm();
            // std::cout << "[SPAR step=" << s_storeCount << " c=" << i << "] "
            //           << "tri=" << m_currentTriSpar[i]
            //           << " theta=" << m_currentTheta[i] << " phi=" << m_currentPhi[i]
            //           << " alpha=" << m_currentAlpha[i] << " beta=" << m_currentBeta[i]
            //           << "\n  rawJac: da_dt=" << da_dt << " db_dt=" << db_dt
            //           << " da_dp=" << da_dp << " db_dp=" << db_dp
            //           << "\n  rowNorm: theta=" << rnT << " phi=" << rnP
            //           << " factor=" << factor
            //           << "\n  lambda_norm: F=(" << lambda[i*5+0] << "," << lambda[i*5+1] << "," << lambda[i*5+2]
            //           << ") slide=(" << lambda[i*5+3] << "," << lambda[i*5+4] << ")"
            //           << "\n  physical: F=(" << Fx << "," << Fy << "," << Fz
            //           << ") dTheta=" << dTheta << " dPhi=" << dPhi
            //           << "\n  smoothF=(" << sF[0] << "," << sF[1] << "," << sF[2]
            //           << ") |smoothF|=" << sFMag
            //           << "\n  pos=" << sphericalToMesh(m_currentTriSpar[i], m_currentAlpha[i], m_currentBeta[i], pos.ref())
            //           << std::endl;
        }

        if (std::isnan(Fx + Fy + Fz + dTheta + dPhi)) continue;

        // Update force
        currentForces[i] = Vec3(Fx, Fy, Fz);

        // EMA: smooth force direction
        Real const momentum = this->d_dirMomentum.getValue();
        if (momentum > 0.0 && m_smoothForces.size() > i)
            m_smoothForces[i] = m_smoothForces[i] * momentum
                              + currentForces[i] * (Real(1.0) - momentum);
        else if (m_smoothForces.size() > i)
            m_smoothForces[i] = currentForces[i];

        // Sliding momentum: EMA of QP slide outputs
        Real const slideMom = this->d_slideMomentum.getValue();
        if (slideMom > 0.0) {
            m_slideMomentumTheta[i] = slideMom * m_slideMomentumTheta[i]
                                    + (Real(1.0) - slideMom) * dTheta;
            m_slideMomentumPhi[i]   = slideMom * m_slideMomentumPhi[i]
                                    + (Real(1.0) - slideMom) * dPhi;
            // Clamp momentum to prevent runaway accumulation
            Real const momClamp = maxStep * Real(2.0);
            m_slideMomentumTheta[i] = std::max(-momClamp, std::min(momClamp, m_slideMomentumTheta[i]));
            m_slideMomentumPhi[i]   = std::max(-momClamp, std::min(momClamp, m_slideMomentumPhi[i]));
            dTheta = m_slideMomentumTheta[i];
            dPhi   = m_slideMomentumPhi[i];
        }

        // Warm-start for next QP
        this->m_lambdaInit[i*s_rowsPerPoint +0] = currentForces[i][0];
        this->m_lambdaInit[i*s_rowsPerPoint +1] = currentForces[i][1];
        this->m_lambdaInit[i*s_rowsPerPoint +2] = currentForces[i][2];
        this->m_lambdaInit[i*s_rowsPerPoint +3] = 0.0;
        this->m_lambdaInit[i*s_rowsPerPoint +4] = 0.0;

        // Apply damping
        dTheta *= damping;
        dPhi   *= damping;

        // Clamp step magnitude (radians)
        Real const stepMag = std::sqrt(dTheta * dTheta + dPhi * dPhi);
        if (stepMag > maxStep) {
            Real const s = maxStep / stepMag;
            dTheta *= s;
            dPhi *= s;
        }

        // Update spherical coordinates
        m_currentTheta[i] += dTheta;
        m_currentPhi[i]   += dPhi;

        // [Claude 2026-04-06] Stagnation detection + random perturbation
        if (stagnationWindow > 0 && i < m_stagnationCount.size()) {
            Real const stepMagAfter = std::sqrt(dTheta * dTheta + dPhi * dPhi);
            if (stepMagAfter < maxStep * Real(0.01)) {
                m_stagnationCount[i]++;
            } else {
                m_stagnationCount[i] = 0;
            }
            if (m_stagnationCount[i] >= stagnationWindow) {
                // Random perturbation seeded from step count + contact index
                unsigned int seed = m_stepCount * 31 + i * 7;
                Real const rTheta = perturbRadius * (Real(2.0) * Real(seed % 1000) / Real(999) - Real(1.0));
                seed = seed * 1103515245 + 12345;
                Real const rPhi   = perturbRadius * (Real(2.0) * Real(seed % 1000) / Real(999) - Real(1.0));
                m_currentTheta[i] += rTheta;
                m_currentPhi[i]   += rPhi;
                m_stagnationCount[i] = 0;
                // Reset momentum to avoid snapping back
                m_slideMomentumTheta[i] = Real(0);
                m_slideMomentumPhi[i]   = Real(0);
            }
        }

        // Clamp theta to (epsilon, pi - epsilon) to avoid pole singularity
        const Real eps = Real(1e-4);
        m_currentTheta[i] = std::max(eps, std::min(Real(M_PI) - eps, m_currentTheta[i]));

        // Wrap phi to [0, 2*pi)
        m_currentPhi[i] = std::fmod(m_currentPhi[i], Real(2.0 * M_PI));
        if (m_currentPhi[i] < 0) m_currentPhi[i] += Real(2.0 * M_PI);

        // Recompute triangle + barycentric on S^Par
        findTriangleOnSphere(m_currentTheta[i], m_currentPhi[i],
                             m_currentTriSpar[i],
                             m_currentAlpha[i], m_currentBeta[i]);
    }

    // Compute world-space contact locations from deformed mesh
    sofa::type::vector<Vec3> currentLocations;
    currentLocations.resize(m_nbContacts);
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        currentLocations[i] = sphericalToMesh(
            m_currentTriSpar[i], m_currentAlpha[i], m_currentBeta[i],
            pos.ref());
    }
    this->d_currentLocation.setValue(currentLocations);
    this->updateLimit();
    Actuator<DataTypes>::storeResults(lambda, delta);
}

// ── Draw ──────────────────────────────────────────────────────────

template<class DataTypes>
void SphericalSlidingForceActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields()
        || !this->d_showForce.getValue()) return;
    if (!this->m_state || m_sparTriangles.empty()) return;
    vparams->drawTool()->setLightingEnabled(true);
    ReadAccessor<Data<VecCoord>> const pos = this->m_state->readPositions();

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        unsigned int const triSpar = m_currentTriSpar[i];
        if (triSpar >= m_sparTriangles.size()) continue;
        const auto& f = m_sparTriangles[triSpar];

        // Highlight current triangle
        if (f[0] < pos.size() && f[1] < pos.size() && f[2] < pos.size()) {
            vparams->drawTool()->drawTriangle(
                pos[f[0]], pos[f[1]], pos[f[2]],
                Vec3(0, 1, 0), sofa::type::RGBAColor::yellow());
        }

        // Contact point
        Vec3 const P = sphericalToMesh(triSpar, m_currentAlpha[i], m_currentBeta[i],
                                 pos.ref());

        Vec3 const force = (i < this->d_currentForces.getValue().size())
                   ? this->d_currentForces.getValue()[i] : Vec3(0, 0, 0);
        if (force.norm2() < s_squaredEpsilon) continue;
        Vec3 const dir = force / force.norm();
        vparams->drawTool()->drawArrow(
            P - dir * std::log(force.norm() + 1) * this->d_visuScale.getValue(),
            P,
            std::log(force.norm() + 1) * this->d_visuScale.getValue() / 20.0,
            sofa::type::RGBAColor::red());
    }
}

} // namespace
