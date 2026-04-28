#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <limits>

#include <SoftRobots.Inverse/component/constraint/AreaContactSlidingForceActuator.h>
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
AreaContactSlidingForceActuator<DataTypes>::AreaContactSlidingForceActuator(MechanicalState* object)
    : Inherit1(object)
    , d_sparFile(initData(&d_sparFile, std::string(""), "sparFile",
                          "Path to .spar binary file (from build_spherical_param.py)"))
    , d_initTheta(initData(&d_initTheta, "initTheta",
                           "Initial theta (polar angle) per contact point"))
    , d_initPhi(initData(&d_initPhi, "initPhi",
                         "Initial phi (azimuthal angle) per contact point"))
    , d_initRadius(initData(&d_initRadius, "initRadius",
                            "Initial contact radius per contact point (mm)"))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max pressure magnitude"))
    , d_minForce(initData(&d_minForce, "minForce", "Min pressure magnitude"))
    , d_initForce(initData(&d_initForce, Vec3(0.0, 0.0, 0.0), "initForce",
                           "Initial pressure guess"))
    , d_maxForceStep(initData(&d_maxForceStep, Real(0.0), "maxForceStep",
                              "Max change in pressure magnitude per iteration (0 = no limit)"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.05), "maxStepSize",
                             "Max (dTheta, dPhi) step per iteration in radians"))
    , d_stepDamping(initData(&d_stepDamping, Real(0.5), "stepDamping",
                             "Damping factor for sliding step"))
    , d_epsilonForce(initData(&d_epsilonForce, Real(1e-3), "epsilonForce",
                              "Regularization for pressure constraint"))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding",
                                "Regularization for sliding constraint"))
    , d_ridgeForce(initData(&d_ridgeForce, Real(1e-12), "ridgeForce",
                            "Ridge for pressure variable"))
    , d_ridgeSliding(initData(&d_ridgeSliding, Real(1e-12), "ridgeSliding",
                              "Ridge for sliding variable"))
    , d_ridgeRadius(initData(&d_ridgeRadius, Real(1e-12), "ridgeRadius",
                             "Ridge for radius variable"))
    , d_epsilonRadius(initData(&d_epsilonRadius, Real(1e-3), "epsilonRadius",
                               "Regularization for radius constraint"))
    , d_jacobianScaleFactor(initData(&d_jacobianScaleFactor, Real(1.0),
                                     "jacobianScaleFactor",
                                     "Factor to scale constraint Jacobian rows"))
    , d_dirMomentum(initData(&d_dirMomentum, Real(0.0), "dirMomentum",
                             "EMA momentum for force direction (0=off, ~0.7=smooth)"))
    , d_slideMomentum(initData(&d_slideMomentum, Real(0.0), "slideMomentum",
                               "EMA momentum for QP sliding output (0=off, ~0.5-0.8=smooth)"))
    , d_sigmoidK(initData(&d_sigmoidK, Real(1.0), "sigmoidK",
                          "Sigmoid sharpness (1/mm). Controls transition width at patch boundary."))
    , d_cutoffThreshold(initData(&d_cutoffThreshold, Real(0.01), "cutoffThreshold",
                                 "Sigmoid values below this are skipped (default 0.01)"))
    , d_maxRadiusStep(initData(&d_maxRadiusStep, Real(1.0), "maxRadiusStep",
                               "Max delta_r per iteration (mm)"))
    , d_minRadius(initData(&d_minRadius, Real(0.5), "minRadius",
                           "Minimum allowed contact radius (mm)"))
    , d_maxRadius(initData(&d_maxRadius, Real(20.0), "maxRadius",
                           "Maximum allowed contact radius (mm)"))
    , d_radiusDamping(initData(&d_radiusDamping, Real(0.3), "radiusDamping",
                               "Damping factor for radius updates"))
    , d_radiusMomentum(initData(&d_radiusMomentum, Real(0.0), "radiusMomentum",
                                "EMA momentum for radius (0=off, ~0.5-0.8=smooth)"))
    , d_currentForces(initData(&d_currentForces, "currentForces",
                               "Current pressure applied"))
    , d_currentLocation(initData(&d_currentLocation, "currentLocation",
                                 "Current force locations in world coordinates"))
    , d_currentRadiusOut(initData(&d_currentRadiusOut, "currentRadius",
                                  "Current contact radius per contact (output)"))
    , d_showForce(initData(&d_showForce, false, "showForce", "Visualize forces"))
    , d_visuScale(initData(&d_visuScale, Real(0.1), "visuScale",
                           "Scale for visualization"))
    , d_topology(initLink("topology", "Mesh topology"))
{
    this->d_showForce.setGroup("Visualization");
    this->d_visuScale.setGroup("Visualization");
}

template<class DataTypes>
AreaContactSlidingForceActuator<DataTypes>::~AreaContactSlidingForceActuator()
{
}

// ── .spar file loader ─────────────────────────────────────────────
// (copied from SphericalSlidingForceActuator — identical binary format)

template<class DataTypes>
bool AreaContactSlidingForceActuator<DataTypes>::loadSparFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        msg_error() << "Cannot open .spar file: " << path;
        return false;
    }

    char magic[4];
    file.read(magic, 4);
    if (magic[0] != 'S' || magic[1] != 'P' || magic[2] != 'A' || magic[3] != 'R') {
        msg_error() << "Invalid .spar file (bad magic): " << path;
        return false;
    }

    uint32_t version, nVerts, nFaces;
    file.read(reinterpret_cast<char*>(&version), 4);
    file.read(reinterpret_cast<char*>(&nVerts), 4);
    file.read(reinterpret_cast<char*>(&nFaces), 4);

    if (version != 1) {
        msg_error() << "Unsupported .spar version: " << version;
        return false;
    }

    msg_info() << "Loading S^Par: " << nVerts << " vertices, " << nFaces << " faces";

    m_sparVertices.resize(nVerts);
    for (uint32_t i = 0; i < nVerts; ++i) {
        double xyz[3];
        file.read(reinterpret_cast<char*>(xyz), 3 * sizeof(double));
        m_sparVertices[i] = Vec3(Real(xyz[0]), Real(xyz[1]), Real(xyz[2]));
    }

    // Skip theta_phi block
    std::streamoff const offset = std::streamoff{nVerts} * 2 * std::streamoff{sizeof(double)};
    file.seekg(offset, std::ios::cur);

    m_sparTriangles.resize(nFaces);
    for (uint32_t i = 0; i < nFaces; ++i) {
        uint32_t idx[3];
        file.read(reinterpret_cast<char*>(idx), 3 * sizeof(uint32_t));
        m_sparTriangles[i] = {idx[0], idx[1], idx[2]};
    }

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
sofa::type::Vec3 AreaContactSlidingForceActuator<DataTypes>::sphericalToCart(Real theta, Real phi)
{
    Real st = std::sin(theta);
    return Vec3(st * std::cos(phi), st * std::sin(phi), std::cos(theta));
}

// ── Radial barycentric (ray from origin) ──────────────────────────

template<class DataTypes>
bool AreaContactSlidingForceActuator<DataTypes>::radialBarycentric(
    const Vec3& v0, const Vec3& v1, const Vec3& v2,
    const Vec3& p_sph, Real& alpha, Real& beta)
{
    Vec3 M = sofa::type::cross(v1 - v0, v2 - v0);
    Real M_dot_p = M * p_sph;
    if (std::abs(M_dot_p) < s_squaredEpsilon) {
        alpha = beta = 0;
        return false;
    }
    Real t = (M * v0) / M_dot_p;
    if (t < 0) {
        alpha = beta = 0;
        return false;
    }

    Vec3 p_plane = p_sph * t;
    Real M_sq = M * M;

    alpha = (M * sofa::type::cross(p_plane - v0, v2 - v0)) / M_sq;
    beta  = (M * sofa::type::cross(v1 - v0, p_plane - v0)) / M_sq;
    return true;
}

// ── Find triangle on S^Par ────────────────────────────────────────

template<class DataTypes>
bool AreaContactSlidingForceActuator<DataTypes>::findTriangleOnSphere(
    Real theta, Real phi,
    unsigned int& triIdx, Real& alpha, Real& beta) const
{
    Vec3 p_sph = sphericalToCart(theta, phi);

    Real bestDist = std::numeric_limits<Real>::max();
    unsigned int bestTri = m_sparTriangles.size();  // sentinel: any value >= size means "not found"
    Real bestAlpha = 0, bestBeta = 0;

    for (unsigned int fi = 0; fi < m_sparTriangles.size(); ++fi) {
        const auto& f = m_sparTriangles[fi];
        Real a, b;
        if (!radialBarycentric(m_sparVertices[f[0]], m_sparVertices[f[1]],
                               m_sparVertices[f[2]], p_sph, a, b))
            continue;

        Real gamma = Real(1.0) - a - b;
        // FP slack on barycentric containment: a/b/gamma can be slightly negative
        // due to round-off when the ray hits exactly on an edge.
        if (a >= Real(-1e-8) && b >= Real(-1e-8) && gamma >= Real(-1e-8)) {
            triIdx = fi;
            alpha = a;
            beta = b;
            return true;
        }

        Real ac = std::max(Real(0), a);
        Real bc = std::max(Real(0), std::min(b, Real(1) - ac));
        Vec3 proj = m_sparVertices[f[0]]
                  + ac * (m_sparVertices[f[1]] - m_sparVertices[f[0]])
                  + bc * (m_sparVertices[f[2]] - m_sparVertices[f[0]]);
        Real pnorm = proj.norm();
        if (pnorm > s_squaredEpsilon) proj /= pnorm;
        Real dist = (proj - p_sph).norm();
        if (dist < bestDist) {
            bestDist = dist;
            bestTri = fi;
            bestAlpha = ac;
            bestBeta = bc;
        }
    }

    if (bestTri >= 0) {
        triIdx = bestTri;
        alpha = bestAlpha;
        beta = bestBeta;
        return true;
    }
    return false;
}

// ── Map S^Par → mesh position (using DEFORMED vertices) ───────────

template<class DataTypes>
sofa::type::Vec3 AreaContactSlidingForceActuator<DataTypes>::sphericalToMesh(
    unsigned int triIdx, Real alpha, Real beta, const VecCoord& pos) const
{
    const auto& f = m_sparTriangles[triIdx];
    Vec3 v0(pos[f[0]]);
    Vec3 v1(pos[f[1]]);
    Vec3 v2(pos[f[2]]);
    return v0 + alpha * (v1 - v0) + beta * (v2 - v0);
}

// ── Sliding Jacobian ──────────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::computeSlidingJacobian(
    unsigned int triIdx, Real theta, Real phi,
    Real& da_dtheta, Real& db_dtheta,
    Real& da_dphi, Real& db_dphi) const
{
    const auto& f = m_sparTriangles[triIdx];
    const Vec3& v0 = m_sparVertices[f[0]];
    const Vec3& v1 = m_sparVertices[f[1]];
    const Vec3& v2 = m_sparVertices[f[2]];

    Vec3 M = sofa::type::cross(v1 - v0, v2 - v0);
    Real M_sq = M * M;
    if (M_sq < s_squaredEpsilon) {
        da_dtheta = db_dtheta = da_dphi = db_dphi = 0;
        return;
    }
    Vec3 M_over_Msq = M / M_sq;

    Real ct = std::cos(theta), st = std::sin(theta);
    Real cp = std::cos(phi),   sp = std::sin(phi);
    Vec3 dP_dtheta(ct * cp, ct * sp, -st);
    Vec3 dP_dphi(-st * sp, st * cp, Real(0));

    Vec3 e1 = v1 - v0;
    Vec3 e2 = v2 - v0;

    da_dtheta = M_over_Msq * sofa::type::cross(dP_dtheta, e2);
    db_dtheta = M_over_Msq * sofa::type::cross(e1, dP_dtheta);
    da_dphi   = M_over_Msq * sofa::type::cross(dP_dphi, e2);
    db_dphi   = M_over_Msq * sofa::type::cross(e1, dP_dphi);
}

// ── Patch computation ─────────────────────────────────────────────
// Called once per simulation step (when m_patchDirty == true).
//
// Two-phase design to prevent visual shaking:
//   Phase 1 (membership rebuild): Only when radius or center moved significantly.
//           Uses a wider search radius (r + margin) so boundary triangles are
//           always included, even after small r changes.
//   Phase 2 (weight update): Every step. Updates C*A and dCdr*A for existing
//           patch triangles using current radius and deformed positions.
//           This is smooth — no triangles pop in/out.

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::recomputePatches(const VecCoord& pos)
{
    Real k = d_sigmoidK.getValue();
    // Margin: include triangles beyond current r so small r changes don't
    // cause membership changes. ~3 sigmoid widths covers C down to ~0.05.
    Real margin = Real(3.0) / ((k > Real(1e-6)) ? k : Real(1.0));

    // Always update centroids and areas from deformed positions
    for (unsigned int fi = 0; fi < m_sparTriangles.size(); ++fi) {
        const auto& f = m_sparTriangles[fi];
        Vec3 v0(pos[f[0]]), v1(pos[f[1]]), v2(pos[f[2]]);
        m_triCentroids[fi] = (v0 + v1 + v2) / Real(3.0);
        m_triAreas[fi] = sofa::type::cross(v1 - v0, v2 - v0).norm() * Real(0.5);
    }

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Vec3 center = sphericalToMesh(m_currentTriSpar[i],
                                       m_currentAlpha[i], m_currentBeta[i], pos);
        Real r = m_currentRadius[i];

        // ── Phase 1: Rebuild triangle membership (rare) ──
        // Only rebuild when radius or center changed enough that the margin
        // can no longer guarantee all relevant triangles are included.
        Real rebuildThreshold = margin * Real(0.5);
        bool needRebuild = m_patches[i].empty()
            || std::abs(r - m_lastPatchRadius[i]) > rebuildThreshold
            || (center - m_lastPatchCenter[i]).norm() > rebuildThreshold;

        if (needRebuild) {
            Real searchRadius = r + margin;
            m_patches[i].clear();
            m_patches[i].reserve(256);

            for (unsigned int fi = 0; fi < m_sparTriangles.size(); ++fi) {
                Real d = (m_triCentroids[fi] - center).norm();
                if (d > searchRadius) continue;
                // Store with placeholder weights (updated in phase 2)
                m_patches[i].push_back({fi, Real(0), Real(0)});
            }

            // Fallback: if patch is empty (r very small), use center triangle
            if (m_patches[i].empty()) {
                m_patches[i].push_back({m_currentTriSpar[i], Real(0), Real(0)});
            }

            m_lastPatchRadius[i] = r;
            m_lastPatchCenter[i] = center;
        }

        // ── Phase 2: Update weights for existing triangles (every step) ──
        // Smooth: no triangles pop in/out, only C*A values change.
        for (auto& pt : m_patches[i]) {
            Real d = (m_triCentroids[pt.triIdx] - center).norm();
            Real C = Real(1.0) / (Real(1.0) + std::exp(k * (d - r)));
            Real A = m_triAreas[pt.triIdx];
            Real dCdr = k * C * (Real(1.0) - C);
            pt.weight = C * A;
            pt.dCdr = dCdr * A;
        }

        // Pre-compute radius row norm (approximate — see plan Section 3.2)
        Real rowNormSq = 0;
        for (const auto& pt : m_patches[i]) {
            bool isCenter = (pt.triIdx == m_currentTriSpar[i]);
            Real wA, wB, wC;
            if (isCenter) {
                wA = Real(1.0) - m_currentAlpha[i] - m_currentBeta[i];
                wB = m_currentAlpha[i];
                wC = m_currentBeta[i];
            } else {
                wA = wB = wC = Real(1.0) / Real(3.0);
            }
            Real g = pt.dCdr;
            rowNormSq += g*g * (wA*wA + wB*wB + wC*wC);
        }
        Real rn = std::sqrt(rowNormSq);
        // Floor at 1e-3 to prevent ill-conditioned QP when sigmoid
        // gradient is near-zero (e.g. radius at min/max or all
        // triangles fully inside/outside the patch).
        m_rowNormR[i] = (rn > Real(1e-3)) ? rn : Real(1e-3);
    }
    m_patchDirty.store(false, std::memory_order_release);
}

// ── Init / Reinit ─────────────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::init()
{
    Inherit1::init();
    if (!this->d_topology.get())
        this->d_topology.set(this->getContext()->getMeshTopology());
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::reinit()
{
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::initData()
{
    // Load S^Par
    const std::string& sparPath = d_sparFile.getValue();
    if (sparPath.empty() || !loadSparFile(sparPath)) {
        msg_error() << "sparFile is required. Generate with build_spherical_param.py";
        return;
    }

    // Determine number of contact points
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

    // 6 DOFs per contact: px, py, pz, dTheta, dPhi, dR
    this->m_dim = m_nbContacts * s_rowsPerContact;
    this->m_nbLines = this->m_dim;

    // Initialize per-contact sliding state
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

    // Initialize radius
    const auto& initR = d_initRadius.getValue();
    m_currentRadius.resize(m_nbContacts);
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        m_currentRadius[i] = (i < initR.size()) ? initR[i] : d_minRadius.getValue();
    }

    // Allocate patch and auxiliary vectors
    m_patches.resize(m_nbContacts);
    m_rowNormR.resize(m_nbContacts, Real(1.0));
    m_lastPatchRadius.assign(m_nbContacts, Real(-1));  // force first rebuild
    m_lastPatchCenter.assign(m_nbContacts, Vec3(0, 0, 0));
    m_triCentroids.resize(m_sparTriangles.size());
    m_triAreas.resize(m_sparTriangles.size());

    m_patchDirty.store(true, std::memory_order_release);

    // Lambda bounds
    this->m_lambdaInit.assign(this->m_dim, 0.0);
    this->m_lambdaMax.assign(this->m_dim, std::numeric_limits<Real>::max());
    this->m_lambdaMin.assign(this->m_dim, std::numeric_limits<Real>::lowest());

    // Initialize forces from face normal
    sofa::type::vector<Vec3> currentForces;
    currentForces.resize(m_nbContacts);
    if (this->d_topology.get() && this->m_state) {
        ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
        Vec3 f0 = this->d_initForce.getValue();
        Real fMag = f0.norm();
        if (fMag == 0.0) fMag = s_fallbackForceMag;
        for (unsigned int i = 0; i < m_nbContacts; ++i) {
            unsigned int triIdx = m_currentTriSpar[i];
            if (triIdx < m_sparTriangles.size()) {
                const auto& f = m_sparTriangles[triIdx];
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

    m_smoothForces = currentForces;

    m_slideMomentumTheta.assign(m_nbContacts, Real(0));
    m_slideMomentumPhi.assign(m_nbContacts, Real(0));
    m_radiusMomentumVal.assign(m_nbContacts, Real(0));

    // Row norms for sliding (initial estimates)
    m_rowNormTheta.assign(m_nbContacts, Real(1));
    m_rowNormPhi.assign(m_nbContacts, Real(1));
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Real da_dt, db_dt, da_dp, db_dp;
        computeSlidingJacobian(m_currentTriSpar[i], m_currentTheta[i], m_currentPhi[i],
                               da_dt, db_dt, da_dp, db_dp);
        Real rnT = std::sqrt((da_dt + db_dt) * (da_dt + db_dt) + da_dt * da_dt + db_dt * db_dt);
        Real rnP = std::sqrt((da_dp + db_dp) * (da_dp + db_dp) + da_dp * da_dp + db_dp * db_dp);
        m_rowNormTheta[i] = (rnT > s_squaredEpsilon) ? rnT : Real(1);
        m_rowNormPhi[i]   = (rnP > s_squaredEpsilon) ? rnP : Real(1);
    }

    // Warm-start lambda
    this->m_hasLambdaInit = true;
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        this->m_lambdaInit[i*s_rowsPerContact +0] = currentForces[i][0];
        this->m_lambdaInit[i*s_rowsPerContact +1] = currentForces[i][1];
        this->m_lambdaInit[i*s_rowsPerContact +2] = currentForces[i][2];
        this->m_lambdaInit[i*s_rowsPerContact +3] = 0.0;  // dTheta
        this->m_lambdaInit[i*s_rowsPerContact +4] = 0.0;  // dPhi
        this->m_lambdaInit[i*s_rowsPerContact +5] = 0.0;  // dR
    }

    // Compute initial world-space locations
    if (this->m_state) {
        ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
        sofa::type::vector<Vec3> currentLocations;
        currentLocations.resize(m_nbContacts);
        for (unsigned int i = 0; i < m_nbContacts; ++i) {
            currentLocations[i] = sphericalToMesh(
                m_currentTriSpar[i], m_currentAlpha[i], m_currentBeta[i],
                pos.ref());
        }
        this->d_currentLocation.setValue(currentLocations);
    }

    // Output radius
    sofa::type::vector<Real> radOut(m_currentRadius.begin(), m_currentRadius.end());
    this->d_currentRadiusOut.setValue(radOut);
}

// ── Update bounds ─────────────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::updateLimit()
{
    Real maxF = this->d_maxForce.isSet() ? this->d_maxForce.getValue() : std::numeric_limits<Real>::max();
    Real minF = this->d_minForce.isSet() ? this->d_minForce.getValue() : std::numeric_limits<Real>::lowest();
    Real maxStep = this->d_maxStepSize.getValue();
    Real maxForceStep = this->d_maxForceStep.getValue();
    Real maxRadStep = this->d_maxRadiusStep.getValue();
    const auto& currentForces = this->d_currentForces.getValue();

    this->m_hasLambdaMax = true;
    this->m_hasLambdaMin = true;

    Real factor = this->d_jacobianScaleFactor.getValue();
    if (factor < s_squaredEpsilon) factor = Real(1);

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Vec3 curF = (i < currentForces.size()) ? currentForces[i] : Vec3(0, 0, 0);

        // Pressure bounds (rows 0-2)
        if (maxForceStep > 0.0) {
            this->m_lambdaMin[i*s_rowsPerContact +0] = std::max(minF, curF[0] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerContact +0] = std::min(maxF, curF[0] + maxForceStep);
            this->m_lambdaMin[i*s_rowsPerContact +1] = std::max(minF, curF[1] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerContact +1] = std::min(maxF, curF[1] + maxForceStep);
            this->m_lambdaMin[i*s_rowsPerContact +2] = std::max(minF, curF[2] - maxForceStep);
            this->m_lambdaMax[i*s_rowsPerContact +2] = std::min(maxF, curF[2] + maxForceStep);
        } else {
            this->m_lambdaMin[i*s_rowsPerContact +0] = minF; this->m_lambdaMax[i*s_rowsPerContact +0] = maxF;
            this->m_lambdaMin[i*s_rowsPerContact +1] = minF; this->m_lambdaMax[i*s_rowsPerContact +1] = maxF;
            this->m_lambdaMin[i*s_rowsPerContact +2] = minF; this->m_lambdaMax[i*s_rowsPerContact +2] = maxF;
        }

        // Sliding bounds (rows 3-4): scaled for normalized rows
        Real rnT = (i < m_rowNormTheta.size()) ? m_rowNormTheta[i] : Real(1);
        Real rnP = (i < m_rowNormPhi.size())   ? m_rowNormPhi[i]   : Real(1);
        Real boundT = maxStep * rnT / factor;
        Real boundP = maxStep * rnP / factor;
        this->m_lambdaMin[i*s_rowsPerContact +3] = -boundT; this->m_lambdaMax[i*s_rowsPerContact +3] = boundT;
        this->m_lambdaMin[i*s_rowsPerContact +4] = -boundP; this->m_lambdaMax[i*s_rowsPerContact +4] = boundP;

        // Radius bounds (row 5): scaled for normalized row
        // Uses m_rowNormR which is computed in recomputePatches (same value as
        // used for invNorm in buildConstraintMatrix, so errors cancel).
        Real rnR = (i < m_rowNormR.size()) ? m_rowNormR[i] : Real(1);
        Real boundR = maxRadStep * rnR / factor;
        this->m_lambdaMin[i*s_rowsPerContact +5] = -boundR; this->m_lambdaMax[i*s_rowsPerContact +5] = boundR;
    }
}

// ── buildConstraintMatrix ─────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::buildConstraintMatrix(
    const ConstraintParams* cParams,
    DataMatrixDeriv &cMatrix,
    unsigned int &cIndex,
    const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    this->d_constraintIndex.setValue(cIndex);
    unsigned int startId = cIndex;
    if (!this->d_topology.get() || m_sparTriangles.empty()) return;

    const VecCoord& pos = x.getValue();
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    Real factor = this->d_jacobianScaleFactor.getValue();

    // Recompute patches once per simulation step
    if (m_patchDirty.load(std::memory_order_acquire))
        recomputePatches(pos);

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        unsigned int triSpar = m_currentTriSpar[i];
        if (triSpar >= m_sparTriangles.size()) continue;
        const auto& patch = m_patches[i];

        // ── Rows 0-2: Pressure (px, py, pz) ──
        // Loop over all patch triangles. Lambda = pressure, Jacobian = C*A*w.
        for (int dim = 0; dim < 3; ++dim) {
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            for (const auto& pt : patch) {
                const auto& f = m_sparTriangles[pt.triIdx];
                bool isCenter = (pt.triIdx == triSpar);
                Real wA, wB, wC;
                if (isCenter) {
                    wA = Real(1.0) - m_currentAlpha[i] - m_currentBeta[i];
                    wB = m_currentAlpha[i];
                    wC = m_currentBeta[i];
                } else {
                    wA = wB = wC = Real(1.0) / Real(3.0);
                }
                Real g = factor * pt.weight;  // factor * C * A
                row.addCol(f[0], Deriv(dim==0?g*wA:0, dim==1?g*wA:0, dim==2?g*wA:0));
                row.addCol(f[1], Deriv(dim==0?g*wB:0, dim==1?g*wB:0, dim==2?g*wB:0));
                row.addCol(f[2], Deriv(dim==0?g*wC:0, dim==1?g*wC:0, dim==2?g*wC:0));
            }
        }

        // ── Rows 3-4: Sliding (dTheta, dPhi) ──
        // Same as SphericalSlidingForceActuator: center triangle only.
        const auto& centerF = m_sparTriangles[triSpar];

        Real da_dtheta, db_dtheta, da_dphi, db_dphi;
        computeSlidingJacobian(triSpar, m_currentTheta[i], m_currentPhi[i],
                               da_dtheta, db_dtheta, da_dphi, db_dphi);

        // Get smooth force direction
        Vec3 smoothF = (m_smoothForces.size() > i)
                     ? m_smoothForces[i] : Vec3(0, 0, 0);
        Real smoothFMag = smoothF.norm();
        if (smoothFMag < Real(1e-6)) {
            Vec3 nFace = sofa::type::cross(
                Vec3(pos[centerF[1]]) - Vec3(pos[centerF[0]]),
                Vec3(pos[centerF[2]]) - Vec3(pos[centerF[0]]));
            Real a2 = nFace.norm();
            smoothF = (a2 > s_squaredEpsilon) ? nFace / a2 : Vec3(0, 0, 1);
        } else {
            smoothF /= smoothFMag;
        }

        // Row 3: dTheta — normalized
        {
            Real rn = std::sqrt(
                (da_dtheta + db_dtheta) * (da_dtheta + db_dtheta)
                + da_dtheta * da_dtheta + db_dtheta * db_dtheta);
            m_rowNormTheta[i] = (rn > s_squaredEpsilon) ? rn : Real(1);
            Real invNorm = Real(1.0) / m_rowNormTheta[i];
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            // No area because it is normalized
            row.addCol(centerF[0], factor * smoothF * (-(da_dtheta + db_dtheta) * invNorm));
            row.addCol(centerF[1], factor * smoothF * (da_dtheta * invNorm));
            row.addCol(centerF[2], factor * smoothF * (db_dtheta * invNorm));
        }

        // Row 4: dPhi — normalized
        {
            Real rn = std::sqrt(
                (da_dphi + db_dphi) * (da_dphi + db_dphi)
                + da_dphi * da_dphi + db_dphi * db_dphi);
            m_rowNormPhi[i] = (rn > s_squaredEpsilon) ? rn : Real(1);
            Real invNorm = Real(1.0) / m_rowNormPhi[i];
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            // No area because it is normalized
            row.addCol(centerF[0], factor * smoothF * (-(da_dphi + db_dphi) * invNorm));
            row.addCol(centerF[1], factor * smoothF * (da_dphi * invNorm));
            row.addCol(centerF[2], factor * smoothF * (db_dphi * invNorm));
        }

        // ── Row 5: Radius (dR) ──
        // Uses pre-computed m_rowNormR from recomputePatches.
        // When the sigmoid gradient is degenerate (all triangles fully
        // inside/outside), write a zero row — the QP ridge/epsilon on
        // this row will keep it well-conditioned.
        {
            Real invNorm = Real(1.0) / m_rowNormR[i];

            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            for (const auto& pt : patch) {
                const auto& f = m_sparTriangles[pt.triIdx];
                bool isCenter = (pt.triIdx == triSpar);
                Real pwA, pwB, pwC;
                if (isCenter) {
                    pwA = Real(1.0) - m_currentAlpha[i] - m_currentBeta[i];
                    pwB = m_currentAlpha[i];
                    pwC = m_currentBeta[i];
                } else {
                    pwA = pwB = pwC = Real(1.0) / Real(3.0);
                }
                Real g = factor * pt.dCdr * invNorm;
                row.addCol(f[0], g * smoothF * pwA);
                row.addCol(f[1], g * smoothF * pwB);
                row.addCol(f[2], g * smoothF * pwC);
            }
        }
    }

    cMatrix.endEdit();
    this->m_nbLines = cIndex - startId;
}

// ── getConstraintViolation ────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::getConstraintViolation(
    const ConstraintParams* cParams, BaseVector *resV, const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams); SOFA_UNUSED(Jdx);
    unsigned int totalDim = m_nbContacts * s_rowsPerContact;
    const auto& constraintId = sofa::helper::getReadAccessor(this->d_constraintIndex);
    for (unsigned int i = 0; i < totalDim; ++i)
        resV->set(constraintId + i, 0.);
}

// ── storeResults ──────────────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::storeResults(
    vector<double> &lambda, vector<double> &delta)
{
    if (!this->d_topology.get() || !this->m_state || m_sparTriangles.empty()) return;
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    WriteAccessor<Data<sofa::type::vector<Vec3>>> currentForces = this->d_currentForces;
    Real damping = this->d_stepDamping.getValue();
    Real maxStep = this->d_maxStepSize.getValue();
    Real radiusDamping = this->d_radiusDamping.getValue();
    Real maxRadStep = this->d_maxRadiusStep.getValue();

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        Real factor = this->d_jacobianScaleFactor.getValue();

        // Extract pressure
        Real px = lambda[i*s_rowsPerContact +0] * factor;
        Real py = lambda[i*s_rowsPerContact +1] * factor;
        Real pz = lambda[i*s_rowsPerContact +2] * factor;

        // Denormalize sliding
        Real rnT = (i < m_rowNormTheta.size() && m_rowNormTheta[i] > s_squaredEpsilon) ? m_rowNormTheta[i] : Real(1);
        Real rnP = (i < m_rowNormPhi.size()   && m_rowNormPhi[i]   > s_squaredEpsilon) ? m_rowNormPhi[i]   : Real(1);
        Real dTheta = lambda[i*s_rowsPerContact +3] * factor / rnT;
        Real dPhi   = lambda[i*s_rowsPerContact +4] * factor / rnP;

        // Denormalize radius
        Real rnR = (i < m_rowNormR.size() && m_rowNormR[i] > s_squaredEpsilon) ? m_rowNormR[i] : Real(1);
        Real dR = lambda[i*s_rowsPerContact +5] * factor / rnR;

        if (std::isnan(px + py + pz + dTheta + dPhi + dR)) continue;

        // Update pressure
        currentForces[i] = Vec3(px, py, pz);

        // EMA: smooth force direction
        Real momentum = this->d_dirMomentum.getValue();
        if (momentum > 0.0 && m_smoothForces.size() > i)
            m_smoothForces[i] = m_smoothForces[i] * momentum
                              + currentForces[i] * (Real(1.0) - momentum);
        else if (m_smoothForces.size() > i)
            m_smoothForces[i] = currentForces[i];

        // Sliding momentum
        Real slideMom = this->d_slideMomentum.getValue();
        if (slideMom > 0.0) {
            m_slideMomentumTheta[i] = slideMom * m_slideMomentumTheta[i]
                                    + (Real(1.0) - slideMom) * dTheta;
            m_slideMomentumPhi[i]   = slideMom * m_slideMomentumPhi[i]
                                    + (Real(1.0) - slideMom) * dPhi;
            Real momClamp = maxStep * Real(2.0);
            m_slideMomentumTheta[i] = std::max(-momClamp, std::min(momClamp, m_slideMomentumTheta[i]));
            m_slideMomentumPhi[i]   = std::max(-momClamp, std::min(momClamp, m_slideMomentumPhi[i]));
            dTheta = m_slideMomentumTheta[i];
            dPhi   = m_slideMomentumPhi[i];
        }

        // Warm-start lambda for next QP
        this->m_lambdaInit[i*s_rowsPerContact +0] = currentForces[i][0];
        this->m_lambdaInit[i*s_rowsPerContact +1] = currentForces[i][1];
        this->m_lambdaInit[i*s_rowsPerContact +2] = currentForces[i][2];
        this->m_lambdaInit[i*s_rowsPerContact +3] = 0.0;
        this->m_lambdaInit[i*s_rowsPerContact +4] = 0.0;
        this->m_lambdaInit[i*s_rowsPerContact +5] = 0.0;

        // Apply damping to sliding
        dTheta *= damping;
        dPhi   *= damping;

        // Clamp sliding step magnitude
        Real stepMag = std::sqrt(dTheta * dTheta + dPhi * dPhi);
        if (stepMag > maxStep) {
            Real s = maxStep / stepMag;
            dTheta *= s;
            dPhi *= s;
        }

        // Update spherical coordinates
        m_currentTheta[i] += dTheta;
        m_currentPhi[i]   += dPhi;

        // Clamp theta to avoid pole singularity
        const Real eps = Real(1e-4);
        m_currentTheta[i] = std::max(eps, std::min(Real(M_PI) - eps, m_currentTheta[i]));

        // Wrap phi to [0, 2*pi)
        m_currentPhi[i] = std::fmod(m_currentPhi[i], Real(2.0 * M_PI));
        if (m_currentPhi[i] < 0) m_currentPhi[i] += Real(2.0 * M_PI);

        // Recompute triangle + barycentric on S^Par
        findTriangleOnSphere(m_currentTheta[i], m_currentPhi[i],
                             m_currentTriSpar[i],
                             m_currentAlpha[i], m_currentBeta[i]);

        // Radius momentum: EMA smoothing (same pattern as slideMomentum)
        Real radMom = this->d_radiusMomentum.getValue();
        if (radMom > 0.0) {
            m_radiusMomentumVal[i] = radMom * m_radiusMomentumVal[i]
                                   + (Real(1.0) - radMom) * dR;
            Real momClamp = maxRadStep * Real(2.0);
            m_radiusMomentumVal[i] = std::max(-momClamp, std::min(momClamp, m_radiusMomentumVal[i]));
            dR = m_radiusMomentumVal[i];
        }

        // Update radius with damping and clamping
        dR *= radiusDamping;
        dR = std::max(-maxRadStep, std::min(maxRadStep, dR));
        m_currentRadius[i] += dR;
        m_currentRadius[i] = std::max(d_minRadius.getValue(),
                             std::min(d_maxRadius.getValue(), m_currentRadius[i]));
    }

    // Mark patches dirty for next step
    m_patchDirty.store(true, std::memory_order_release);

    // Compute world-space contact locations
    sofa::type::vector<Vec3> currentLocations;
    currentLocations.resize(m_nbContacts);
    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        currentLocations[i] = sphericalToMesh(
            m_currentTriSpar[i], m_currentAlpha[i], m_currentBeta[i],
            pos.ref());
    }
    this->d_currentLocation.setValue(currentLocations);

    // Output current radius
    sofa::type::vector<Real> radOut(m_currentRadius.begin(), m_currentRadius.end());
    this->d_currentRadiusOut.setValue(radOut);

    this->updateLimit();
    Actuator<DataTypes>::storeResults(lambda, delta);
}

// ── Draw ──────────────────────────────────────────────────────────

template<class DataTypes>
void AreaContactSlidingForceActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields()
        || !this->d_showForce.getValue()) return;
    if (!this->m_state || m_sparTriangles.empty()) return;
    vparams->drawTool()->setLightingEnabled(true);
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();

    for (unsigned int i = 0; i < m_nbContacts; ++i) {
        unsigned int triSpar = m_currentTriSpar[i];
        if (triSpar >= m_sparTriangles.size()) continue;

        Vec3 center = sphericalToMesh(triSpar, m_currentAlpha[i], m_currentBeta[i],
                                       pos.ref());

        // 1. Patch triangles: colored by sigmoid weight C
        //    Green (C≈1, interior) → Yellow (C≈0.5, boundary) → Red (C≈0, edge)
        for (const auto& pt : m_patches[i]) {
            const auto& f = m_sparTriangles[pt.triIdx];
            if (f[0] >= pos.size() || f[1] >= pos.size() || f[2] >= pos.size()) continue;
            Vec3 v0(pos[f[0]]), v1(pos[f[1]]), v2(pos[f[2]]);
            Real A = m_triAreas[pt.triIdx];
            Real C = (A > s_squaredEpsilon) ? pt.weight / A : Real(0);  // recover C from stored C*A
            sofa::type::RGBAColor color(1.0f - float(C), float(C), 0.0f, 0.4f);
            Vec3 n = sofa::type::cross(v1 - v0, v2 - v0);
            Real nl = n.norm();
            if (nl > s_squaredEpsilon) n /= nl;
            vparams->drawTool()->drawTriangle(v0, v1, v2, n, color);
        }

        // 2. Contact center: white sphere
        vparams->drawTool()->drawSphere(center, 0.5);

        // 3. Current force vector: blue arrow from center
        Vec3 forceDir = (i < m_smoothForces.size()) ? m_smoothForces[i] : Vec3(0,0,0);
        Real fMag = forceDir.norm();
        if (fMag < s_squaredEpsilon) continue;
        Real scale = d_visuScale.getValue();
        vparams->drawTool()->drawArrow(
            center,
            center + forceDir / fMag * std::log(fMag + 1) * scale,
            std::log(fMag + 1) * scale / 20.0,
            sofa::type::RGBAColor::blue());
    }
}

} // namespace
