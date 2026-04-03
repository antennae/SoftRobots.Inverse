#pragma once

#include <unordered_set>
#include <iostream>

#include <SoftRobots.Inverse/component/constraint/SmoothSlidingForceActuator.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/geometry/proximity/PointTriangle.h>

namespace softrobotsinverse::constraint
{
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::type::Vec3;
using sofa::core::objectmodel::Data ;
using sofa::type::vector;
using sofa::linearalgebra::BaseVector;

template<class DataTypes>
SmoothSlidingForceActuator<DataTypes>::SmoothSlidingForceActuator(MechanicalState* object)
    : Inherit1(object)
    , d_triangleIndices(initData(&d_triangleIndices, "triangleIndices", "Indices of active triangles"))
    , d_localCoords(initData(&d_localCoords, "localCoords", "Local Cartesian coords (U, V, 0) in tangent plane for each point."))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max normal force"))
    , d_minForce(initData(&d_minForce, "minForce", "Min normal force"))
    , d_initForce(initData(&d_initForce, Vec3(0.0, 0.0, 0.0), "initForce", "Initial force guess"))
    , d_maxForceStep(initData(&d_maxForceStep, Real(0.0), "maxForceStep", "Max change in force magnitude per iteration (0 = no limit)"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.1), "maxStepSize", "Trust region for sliding (Cartesian step limit in tangent plane)"))
    , d_stepDamping(initData(&d_stepDamping, Real(0.5), "stepDamping", "Damping factor for sliding step (0.1). Lower reduces jitter."))
    , d_epsilonForce(initData(&d_epsilonForce, Real(1e-3), "epsilonForce", "Regularization for force constraint."))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding", "Regularization for sliding constraint."))
    , d_ridgeForce(initData(&d_ridgeForce, Real(1e-12), "ridgeForce", "Ridge for force variable."))
    , d_ridgeSliding(initData(&d_ridgeSliding, Real(1e-12), "ridgeSliding", "Ridge for sliding variable."))
    , d_jacobianScaleFactor(initData(&d_jacobianScaleFactor, Real(1.0), "jacobianScaleFactor", "Factor to scale constraint Jacobian rows."))
    , d_dirMomentum(initData(&d_dirMomentum, Real(0.0), "dirMomentum", "EMA momentum for the force direction used in the sliding Jacobian (0=off, ~0.7=smooth). Breaks the force-direction feedback loop."))
    , d_currentForces(initData(&d_currentForces, "currentForces", "Current forces applied"))
    , d_currentLocation(initData(&d_currentLocation, "currentLocation", "Current force locations in world coordinates"))
    , d_showForce(initData(&d_showForce, false, "showForce", "Visualize forces"))
    , d_visuScale(initData(&d_visuScale, Real(0.1), "visuScale", "Scale for visualization"))
    , d_topology(initLink("topology", "Mesh topology"))
{
    this->d_showForce.setGroup("Visualization");
    this->d_visuScale.setGroup("Visualization");
}

template<class DataTypes>
SmoothSlidingForceActuator<DataTypes>::~SmoothSlidingForceActuator()
{
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::init()
{
    Inherit1::init();
    if (!this->d_topology.get()) this->d_topology.set(this->getContext()->getMeshTopology());
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::reinit()
{
    this->initData();
    this->updateLimit();
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::initData()
{
    unsigned int nbPoints = this->d_triangleIndices.getValue().size();
    this->m_dim = nbPoints * 5; 
    this->m_nbLines = this->m_dim;
    this->m_activeTriangles = this->d_triangleIndices.getValue();
    
    if(this->d_localCoords.getValue().size() == nbPoints) {
        this->m_activeLocalCoords = this->d_localCoords.getValue();
    } else {
        this->m_activeLocalCoords.assign(nbPoints, sofa::type::Vec3(0,0,0));
        const auto& triangles = (this->d_topology.get()) ? this->d_topology.get()->getTriangles() : sofa::type::vector<Triangle>();
        if (this->m_state && !triangles.empty()) {
             ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
             for(unsigned int i=0; i<nbPoints; i++) {
                 unsigned int triIdx = this->m_activeTriangles[i];
                 if(triIdx < triangles.size()) {
                     const Triangle& t = triangles[triIdx];
                     Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
                     Coord P = (A+B+C)/3.0;
                     Deriv v1 = B-A; sofa::type::Vec3 e1 = v1; e1.normalize();
                     sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                     sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
                     this->m_activeLocalCoords[i][0] = (P-A)*e1; this->m_activeLocalCoords[i][1] = (P-A)*e2;
                 }
             }
        }
    }

    this->m_lambdaInit.assign(this->m_dim, 0.0);
    this->m_lambdaMax.assign(this->m_dim, 1e20);
    this->m_lambdaMin.assign(this->m_dim, -1e20);

    sofa::type::vector<sofa::type::Vec3> currentForces;
    currentForces.resize(nbPoints);
    if (this->d_topology.get() && this->m_state) {
        const auto& triangles = this->d_topology.get()->getTriangles();
        ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
        sofa::type::Vec3 f0 = this->d_initForce.getValue();
        Real fMag = f0.norm(); if (fMag == 0.0) fMag = 1e-3;
        for(unsigned int i=0; i<nbPoints; i++) {
             unsigned int triIdx = this->m_activeTriangles[i];
             if(triIdx < triangles.size()) {
                 const Triangle& t = triangles[triIdx];
                 sofa::type::Vec3 n = sofa::type::cross(pos[t[1]]-pos[t[0]], pos[t[2]]-pos[t[0]]);
                 n.normalize(); currentForces[i] = n * fMag;
             }
        }
    }
    this->d_currentForces.setValue(currentForces);

    // Initialise smooth force directions (used for Jacobian) to same value
    this->m_smoothForces = currentForces;

    // Warm-start: seed lambdaInit from initForce so the QP starts near the solution
    this->m_hasLambdaInit = true;
    for (unsigned int i = 0; i < nbPoints; i++) {
        this->m_lambdaInit[i*5 + 0] = currentForces[i][0];
        this->m_lambdaInit[i*5 + 1] = currentForces[i][1];
        this->m_lambdaInit[i*5 + 2] = currentForces[i][2];
        this->m_lambdaInit[i*5 + 3] = 0.0;
        this->m_lambdaInit[i*5 + 4] = 0.0;
    }
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::updateVertexNormals()
{
    if (!this->d_topology.get() || !this->m_state) return;
    const auto& triangles = this->d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    size_t nbNodes = pos.size();
    this->m_vertexNormals.assign(nbNodes, sofa::type::Vec3(0,0,0));
    for (const auto& tri : triangles) {
        sofa::type::Vec3 n = sofa::type::cross(pos[tri[1]]-pos[tri[0]], pos[tri[2]]-pos[tri[0]]);
        this->m_vertexNormals[tri[0]] += n; this->m_vertexNormals[tri[1]] += n; this->m_vertexNormals[tri[2]] += n;
    }
    for (auto& n : this->m_vertexNormals) if (n.norm2() > 1e-12) n.normalize();
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::updateLimit()
{
    Real maxF = this->d_maxForce.isSet() ? this->d_maxForce.getValue() : 1e20;
    Real minF = this->d_minForce.isSet() ? this->d_minForce.getValue() : -1e20;
    Real maxStep = this->d_maxStepSize.getValue();
    Real maxForceStep = this->d_maxForceStep.getValue();
    const auto& currentForces = this->d_currentForces.getValue();
    unsigned int nbPoints = this->m_activeTriangles.size();

    for(unsigned int i=0; i<nbPoints; i++) {
        sofa::type::Vec3 curF = (i < currentForces.size()) ? currentForces[i] : sofa::type::Vec3(0,0,0);

        if (maxForceStep > 0.0) {
            this->m_lambdaMin[i*5 + 0] = std::max(minF, curF[0] - maxForceStep);
            this->m_lambdaMax[i*5 + 0] = std::min(maxF, curF[0] + maxForceStep);
            this->m_lambdaMin[i*5 + 1] = std::max(minF, curF[1] - maxForceStep);
            this->m_lambdaMax[i*5 + 1] = std::min(maxF, curF[1] + maxForceStep);
            this->m_lambdaMin[i*5 + 2] = std::max(minF, curF[2] - maxForceStep);
            this->m_lambdaMax[i*5 + 2] = std::min(maxF, curF[2] + maxForceStep);
        } else {
            this->m_lambdaMin[i*5 + 0] = minF; this->m_lambdaMax[i*5 + 0] = maxF;
            this->m_lambdaMin[i*5 + 1] = minF; this->m_lambdaMax[i*5 + 1] = maxF;
            this->m_lambdaMin[i*5 + 2] = minF; this->m_lambdaMax[i*5 + 2] = maxF;
        }
        // Sliding bounds: plain Cartesian step limit, no fMag scaling
        this->m_lambdaMin[i*5 + 3] = -maxStep; this->m_lambdaMax[i*5 + 3] = maxStep;
        this->m_lambdaMin[i*5 + 4] = -maxStep; this->m_lambdaMax[i*5 + 4] = maxStep;
    }
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                               DataMatrixDeriv &cMatrix,
                                                               unsigned int &cIndex,
                                                               const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    this->d_constraintIndex.setValue(cIndex);
    unsigned int startId = cIndex;
    if (!this->d_topology.get()) return;
    const auto& triangles = this->d_topology.get()->getTriangles();
    const VecCoord& pos = x.getValue();
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    
    for(unsigned int i=0; i<this->m_activeTriangles.size(); i++) {
        unsigned int triIdx = this->m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;
        const Triangle& tri = triangles[triIdx];
        sofa::type::Vec3 local = this->m_activeLocalCoords[i];
        const Coord& A = pos[tri[0]]; const Coord& B = pos[tri[1]]; const Coord& C = pos[tri[2]];
        
        Deriv v1 = B - A; Deriv v2 = C - A;
        sofa::type::Vec3 nFace = sofa::type::cross(v1, v2); Real area2 = nFace.norm(); if (area2 > 1e-12) nFace /= area2;
        sofa::type::Vec3 e1Face = v1; e1Face.normalize();
        sofa::type::Vec3 e2Face = sofa::type::cross(nFace, e1Face);

        Real v1x = v1 * e1Face; Real v2x = v2 * e1Face; Real v2y = v2 * e2Face;
        if (std::abs(v1x) < 1e-12) v1x = 1.0; if (std::abs(v2y) < 1e-12) v2y = 1.0;
        Real wC = local[1] / v2y; Real wB = (local[0] - wC * v2x) / v1x; Real wA = 1.0 - wB - wC;

        Real factor = this->d_jacobianScaleFactor.getValue();

        // --- Rows 0, 1, 2: Force components (Fx, Fy, Fz) ---
        // Identical to SlidingForceActuator: barycentric interpolation of identity
        for (int dim=0; dim<3; ++dim) {
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            row.addCol(tri[0], factor * Deriv(dim==0?wA:0, dim==1?wA:0, dim==2?wA:0));
            row.addCol(tri[1], factor * Deriv(dim==0?wB:0, dim==1?wB:0, dim==2?wB:0));
            row.addCol(tri[2], factor * Deriv(dim==0?wC:0, dim==1?wC:0, dim==2?wC:0));
        }

        // --- Rows 3, 4: Sliding (dU, dV) ---
        // Use the SAME weight-derivative structure as the original SlidingForceActuator
        // (tangent-plane derivatives of barycentric weights), but with the EMA-filtered
        // force direction as the coupling vector instead of the instantaneous force.
        Real det = v1x * v2y;
        Real du_dU = 1.0 / v1x;
        Real du_dV = -v2x / det;
        Real dv_dU = 0.0;
        Real dv_dV = 1.0 / v2y;

        // Smooth force direction for the scaling vector (breaks feedback loop)
        sofa::type::Vec3 smoothF = (this->m_smoothForces.size() > i)
                                   ? this->m_smoothForces[i]
                                   : sofa::type::Vec3(0,0,0);
        Real smoothFMag = smoothF.norm();
        if (smoothFMag < 1e-6) { smoothF = nFace; smoothFMag = 1.0; }
        sofa::type::Vec3 scaledGradient = smoothF / smoothFMag;

        // Row 3: Sliding U (dU)
        MatrixDerivRowIterator rowSlideU = matrix.writeLine(cIndex++);
        rowSlideU.addCol(tri[0], factor * (-(du_dU + dv_dU)) * scaledGradient);
        rowSlideU.addCol(tri[1], factor * du_dU * scaledGradient);
        rowSlideU.addCol(tri[2], factor * dv_dU * scaledGradient);

        // Row 4: Sliding V (dV)
        MatrixDerivRowIterator rowSlideV = matrix.writeLine(cIndex++);
        rowSlideV.addCol(tri[0], factor * (-(du_dV + dv_dV)) * scaledGradient);
        rowSlideV.addCol(tri[1], factor * du_dV * scaledGradient);
        rowSlideV.addCol(tri[2], factor * dv_dV * scaledGradient);
    }
    cMatrix.endEdit();
    this->m_nbLines = cIndex - startId;
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams, BaseVector *resV, const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams); SOFA_UNUSED(Jdx);
    unsigned int totalDim = this->m_activeTriangles.size() * 5;
    const auto& constraintId = sofa::helper::getReadAccessor(this->d_constraintIndex);
    for(unsigned int i=0; i<totalDim; i++) resV->set(constraintId + i, 0.);
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::projectToMesh(unsigned int& triIdx, sofa::type::Vec3& local)
{
    if (!this->d_topology.get() || !this->m_state) return;
    const auto& triangles = this->d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    if (triIdx >= triangles.size()) return;
    const Triangle& t = triangles[triIdx];
    Coord A0 = pos[t[0]]; Coord B0 = pos[t[1]]; Coord C0 = pos[t[2]];
    Deriv v1_0 = B0 - A0; sofa::type::Vec3 e1_0 = v1_0; e1_0.normalize();
    sofa::type::Vec3 n_0 = sofa::type::cross(B0-A0, C0-A0); n_0.normalize();
    sofa::type::Vec3 e2_0 = sofa::type::cross(n_0, e1_0);
    Coord candidatePos = A0 + e1_0 * local[0] + e2_0 * local[1];
    Real minDist = 1e99; int bestTri = -1; sofa::type::Vec3 bestLocal;
    for(unsigned int i=0; i<triangles.size(); i++) {
        sofa::type::Vec3 close; const Triangle& tri = triangles[i];
        if (sofa::geometry::proximity::computeClosestPointOnTriangleToPoint(sofa::type::Vec3(pos[tri[0]]), sofa::type::Vec3(pos[tri[1]]), sofa::type::Vec3(pos[tri[2]]), sofa::type::Vec3(candidatePos), close)) {
            Real d = (Coord(close) - candidatePos).norm();
            if (d < minDist) {
                minDist = d; bestTri = i;
                Coord A = pos[tri[0]]; Coord B = pos[tri[1]]; Coord C = pos[tri[2]];
                Deriv v1 = B - A; sofa::type::Vec3 e1 = v1; e1.normalize();
                sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
                bestLocal[0] = (Coord(close) - A) * e1; bestLocal[1] = (Coord(close) - A) * e2; bestLocal[2] = 0;
            }
        }
    }
    if (bestTri != -1) { triIdx = bestTri; local = bestLocal; }
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    unsigned int nbPoints = this->m_activeTriangles.size();
    if (!this->d_topology.get() || !this->m_state) return;
    const auto& triangles = this->d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    WriteAccessor<Data<sofa::type::vector<sofa::type::Vec3>>> currentForces = this->d_currentForces;
    Real maxStep = this->d_maxStepSize.getValue(); Real damping = this->d_stepDamping.getValue();
    for(unsigned int i=0; i<nbPoints; i++) {
        Real Fx = lambda[i*5 + 0]; Real Fy = lambda[i*5 + 1]; Real Fz = lambda[i*5 + 2];
        Real dU_weighted = lambda[i*5 + 3]; Real dV_weighted = lambda[i*5 + 4];
        if (std::isnan(Fx+Fy+Fz+dU_weighted+dV_weighted)) continue;
        currentForces[i] = sofa::type::Vec3(Fx, Fy, Fz);

        // --- EMA: update smooth force direction used by next Jacobian build ---
        Real momentum = this->d_dirMomentum.getValue();
        if (momentum > 0.0 && this->m_smoothForces.size() > i)
            this->m_smoothForces[i] = this->m_smoothForces[i] * momentum
                                      + currentForces[i] * (1.0 - momentum);
        else if (this->m_smoothForces.size() > i)
            this->m_smoothForces[i] = currentForces[i];

        // --- Warm-start: next QP begins from current solution, not stale initForce ---
        this->m_lambdaInit[i*5 + 0] = currentForces[i][0];
        this->m_lambdaInit[i*5 + 1] = currentForces[i][1];
        this->m_lambdaInit[i*5 + 2] = currentForces[i][2];
        this->m_lambdaInit[i*5 + 3] = 0.0;
        this->m_lambdaInit[i*5 + 4] = 0.0;

        unsigned int triIdx = this->m_activeTriangles[i];
        const Triangle& tri = triangles[triIdx];
        Coord A = pos[tri[0]]; Coord B = pos[tri[1]]; Coord C = pos[tri[2]];
        Deriv v1 = B - A; Deriv v2 = C - A;
        sofa::type::Vec3 nFace = sofa::type::cross(v1, v2); nFace.normalize();
        sofa::type::Vec3 e1Face = v1; e1Face.normalize();
        sofa::type::Vec3 e2Face = sofa::type::cross(nFace, e1Face);

        // Sliding variables are plain Cartesian steps in the face tangent plane
        // (matches the Jacobian which uses face-local du_dU/dv_dV derivatives)
        Real dU = dU_weighted; Real dV = dV_weighted;

        dU *= damping; dV *= damping;
        if (std::sqrt(dU*dU + dV*dV) > maxStep) { Real s = maxStep / std::sqrt(dU*dU+dV*dV); dU *= s; dV *= s; }

        // Update local coords directly in the face tangent frame
        this->m_activeLocalCoords[i][0] += dU;
        this->m_activeLocalCoords[i][1] += dV;

        // Check OOB via barycentric weights and project if needed
        Real v1x = v1 * e1Face; Real v2x = v2 * e1Face; Real v2y = v2 * e2Face;
        if (std::abs(v1x) < 1e-12) v1x = 1.0; if (std::abs(v2y) < 1e-12) v2y = 1.0;
        Real wC = this->m_activeLocalCoords[i][1] / v2y;
        Real wB = (this->m_activeLocalCoords[i][0] - wC * v2x) / v1x;
        Real wA = 1.0 - wB - wC;
        if (wA < 0 || wB < 0 || wC < 0)
            this->projectToMesh(this->m_activeTriangles[i], this->m_activeLocalCoords[i]);
    }
    this->d_triangleIndices.setValue(this->m_activeTriangles);
    this->d_localCoords.setValue(this->m_activeLocalCoords);

    sofa::type::vector<sofa::type::Vec3> currentLocations; currentLocations.resize(nbPoints);
    for(unsigned int i=0; i<nbPoints; i++) {
        unsigned int triIdx = this->m_activeTriangles[i];
        if(triIdx < triangles.size()) {
            const Triangle& t = triangles[triIdx];
            Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
            Deriv v1 = B-A; sofa::type::Vec3 e1 = v1; e1.normalize();
            sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
            sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
            currentLocations[i] = A + e1 * this->m_activeLocalCoords[i][0] + e2 * this->m_activeLocalCoords[i][1];
        }
    }
    this->d_currentLocation.setValue(currentLocations);
    this->updateLimit();
    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields() || !this->d_showForce.getValue()) return;
    if (!this->d_topology.get() || !this->m_state) return;
    vparams->drawTool()->setLightingEnabled(true);
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    const auto& triangles = this->d_topology.get()->getTriangles();
    for(unsigned int i=0; i<this->m_activeTriangles.size(); i++) {
        unsigned int triIdx = this->m_activeTriangles[i]; if (triIdx >= triangles.size()) continue;
        const Triangle& t = triangles[triIdx];
        vparams->drawTool()->drawTriangle(pos[t[0]], pos[t[1]], pos[t[2]], sofa::type::Vec3(0,1,0), sofa::type::RGBAColor::yellow());
        sofa::type::Vec3 local = this->m_activeLocalCoords[i];
        Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
        Deriv v1 = B - A; sofa::type::Vec3 e1 = v1; e1.normalize();
        sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
        sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
        Coord P = A + e1 * local[0] + e2 * local[1];
        sofa::type::Vec3 f = this->d_currentForces.getValue().size() > i ? this->d_currentForces.getValue()[i] : sofa::type::Vec3(0,0,0);
        if (f.norm2() < 1e-12) continue;
        sofa::type::Vec3 dir = f/f.norm();
        vparams->drawTool()->drawArrow(P - dir * log(f.norm()+1)*this->d_visuScale.getValue(), P, log(f.norm()+1)*this->d_visuScale.getValue()/20.0, sofa::type::RGBAColor::red());
    }
}

} // namespace
