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
    , d_localCoords(initData(&d_localCoords, "localCoords", "Barycentric coords (wB, wC, 0) for each contact point."))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max normal force"))
    , d_minForce(initData(&d_minForce, "minForce", "Min normal force"))
    , d_initForce(initData(&d_initForce, Vec3(0.0, 0.0, 0.0), "initForce", "Initial force guess"))
    , d_maxForceStep(initData(&d_maxForceStep, Real(0.0), "maxForceStep", "Max change in force magnitude per iteration (0 = no limit)"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.1), "maxStepSize", "Max sliding step in mm per iteration"))
    , d_stepDamping(initData(&d_stepDamping, Real(0.5), "stepDamping", "Damping factor for sliding step (0.1). Lower reduces jitter."))
    , d_epsilonForce(initData(&d_epsilonForce, Real(1e-3), "epsilonForce", "Regularization for force constraint."))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding", "Regularization for sliding constraint."))
    , d_ridgeForce(initData(&d_ridgeForce, Real(1e-12), "ridgeForce", "Ridge for force variable."))
    , d_ridgeSliding(initData(&d_ridgeSliding, Real(1e-12), "ridgeSliding", "Ridge for sliding variable."))
    , d_jacobianScaleFactor(initData(&d_jacobianScaleFactor, Real(1.0), "jacobianScaleFactor", "Factor to scale constraint Jacobian rows."))
    , d_dirMomentum(initData(&d_dirMomentum, Real(0.0), "dirMomentum", "EMA momentum for the force direction used in the sliding Jacobian (0=off, ~0.7=smooth)."))
    , d_slideMomentum(initData(&d_slideMomentum, Real(0.0), "slideMomentum", "EMA momentum for QP sliding output (0=off, ~0.5-0.8=smooth). Filters noisy dwB/dwC so only consistent slide directions accumulate."))
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

    // State: m_activeLocalCoords[i] = (wB, wC, 0) — barycentric coordinates.
    // wA = 1 - wB - wC. Frame-independent, numerically stable.
    if(this->d_localCoords.getValue().size() == nbPoints) {
        this->m_activeLocalCoords = this->d_localCoords.getValue();
    } else {
        // Default: centroid of each triangle (wA = wB = wC = 1/3)
        this->m_activeLocalCoords.assign(nbPoints, sofa::type::Vec3(1.0/3, 1.0/3, 0));
    }

    // Compute mean edge length for step-size conversion (mm -> barycentric)
    if (this->d_topology.get() && this->m_state) {
        const auto& triangles = this->d_topology.get()->getTriangles();
        ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
        Real sumLen = 0.0; int count = 0;
        for (const auto& tri : triangles) {
            sumLen += (pos[tri[1]] - pos[tri[0]]).norm();
            sumLen += (pos[tri[2]] - pos[tri[1]]).norm();
            sumLen += (pos[tri[0]] - pos[tri[2]]).norm();
            count += 3;
        }
        m_meanEdgeLength = (count > 0 && sumLen > 1e-12) ? sumLen / count : 1.0;
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

    // Initialise sliding momentum accumulators to zero
    this->m_slideMomentumB.assign(nbPoints, Real(0));
    this->m_slideMomentumC.assign(nbPoints, Real(0));

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
    // Convert mm step size to barycentric units
    Real maxStep_bary = (m_meanEdgeLength > 1e-12)
                        ? this->d_maxStepSize.getValue() / m_meanEdgeLength
                        : this->d_maxStepSize.getValue();
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
        // Sliding bounds in barycentric units
        this->m_lambdaMin[i*5 + 3] = -maxStep_bary; this->m_lambdaMax[i*5 + 3] = maxStep_bary;
        this->m_lambdaMin[i*5 + 4] = -maxStep_bary; this->m_lambdaMax[i*5 + 4] = maxStep_bary;
    }
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::getBarycentricCoords(
    const sofa::type::Vec3& A, const sofa::type::Vec3& B, const sofa::type::Vec3& C,
    const sofa::type::Vec3& P, Real& wB, Real& wC)
{
    // Möller barycentric decomposition (frame-independent)
    sofa::type::Vec3 v0 = B - A;  // edge AB
    sofa::type::Vec3 v1 = C - A;  // edge AC
    sofa::type::Vec3 v2 = P - A;
    Real d00 = v0 * v0;
    Real d01 = v0 * v1;
    Real d11 = v1 * v1;
    Real d20 = v2 * v0;
    Real d21 = v2 * v1;
    Real denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-12) { wB = 1.0/3; wC = 1.0/3; return; }
    wB = (d11 * d20 - d01 * d21) / denom;
    wC = (d00 * d21 - d01 * d20) / denom;
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

    Real factor = this->d_jacobianScaleFactor.getValue();

    for(unsigned int i=0; i<this->m_activeTriangles.size(); i++) {
        unsigned int triIdx = this->m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;
        const Triangle& tri = triangles[triIdx];
        const sofa::type::Vec3& local = this->m_activeLocalCoords[i];

        // Barycentric weights: local = (wB, wC, 0)
        Real wB = local[0], wC = local[1], wA = 1.0 - wB - wC;

        // --- Rows 0, 1, 2: Force components (Fx, Fy, Fz) ---
        // Barycentric interpolation of identity: each component gets wA/wB/wC weight
        for (int dim = 0; dim < 3; ++dim) {
            MatrixDerivRowIterator row = matrix.writeLine(cIndex++);
            row.addCol(tri[0], factor * Deriv(dim==0?wA:0, dim==1?wA:0, dim==2?wA:0));
            row.addCol(tri[1], factor * Deriv(dim==0?wB:0, dim==1?wB:0, dim==2?wB:0));
            row.addCol(tri[2], factor * Deriv(dim==0?wC:0, dim==1?wC:0, dim==2?wC:0));
        }

        // --- Rows 3, 4: Sliding (d/dwB, d/dwC) ---
        // P = wA*A + wB*B + wC*C = A + wB*(B-A) + wC*(C-A)
        // dP/dwB = B - A, so force shifts from tri[0] to tri[1]: coeffs {-1, +1,  0}
        // dP/dwC = C - A, so force shifts from tri[0] to tri[2]: coeffs {-1,  0, +1}
        // Scale by force direction so the optimizer moves the contact point.
        sofa::type::Vec3 smoothF = (this->m_smoothForces.size() > i)
                                   ? this->m_smoothForces[i]
                                   : sofa::type::Vec3(0, 0, 0);
        Real smoothFMag = smoothF.norm();
        if (smoothFMag < 1e-6) {
            // Fall back to face normal
            sofa::type::Vec3 nFace = sofa::type::cross(
                pos[tri[1]] - pos[tri[0]], pos[tri[2]] - pos[tri[0]]);
            Real a2 = nFace.norm();
            smoothF = (a2 > 1e-12) ? nFace / a2 : sofa::type::Vec3(0, 0, 1);
        } else {
            smoothF /= smoothFMag;
        }

        // Row 3: d/dwB
        MatrixDerivRowIterator rowSlideB = matrix.writeLine(cIndex++);
        rowSlideB.addCol(tri[0], -factor * smoothF);
        rowSlideB.addCol(tri[1],  factor * smoothF);

        // Row 4: d/dwC
        MatrixDerivRowIterator rowSlideC = matrix.writeLine(cIndex++);
        rowSlideC.addCol(tri[0], -factor * smoothF);
        rowSlideC.addCol(tri[2],  factor * smoothF);
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

    // Reconstruct 3D candidate from barycentric
    const Triangle& t0 = triangles[triIdx];
    Real wB = local[0], wC = local[1], wA = 1.0 - wB - wC;
    sofa::type::Vec3 candidatePos =
        sofa::type::Vec3(pos[t0[0]]) * wA +
        sofa::type::Vec3(pos[t0[1]]) * wB +
        sofa::type::Vec3(pos[t0[2]]) * wC;

    // Find closest point on mesh surface
    Real minDist = 1e99;
    int bestTri = -1;
    sofa::type::Vec3 bestClose;
    for(unsigned int j = 0; j < triangles.size(); j++) {
        sofa::type::Vec3 close;
        const Triangle& tri = triangles[j];
        if (sofa::geometry::proximity::computeClosestPointOnTriangleToPoint(
                sofa::type::Vec3(pos[tri[0]]), sofa::type::Vec3(pos[tri[1]]),
                sofa::type::Vec3(pos[tri[2]]), candidatePos, close)) {
            Real d = (close - candidatePos).norm();
            if (d < minDist) { minDist = d; bestTri = j; bestClose = close; }
        }
    }
    if (bestTri < 0) return;

    // Compute barycentric coords of the closest point
    const Triangle& bt = triangles[bestTri];
    Real newWB, newWC;
    getBarycentricCoords(sofa::type::Vec3(pos[bt[0]]), sofa::type::Vec3(pos[bt[1]]),
                         sofa::type::Vec3(pos[bt[2]]), bestClose, newWB, newWC);
    // Clamp to valid range
    newWB = std::max(Real(0), newWB);
    newWC = std::max(Real(0), newWC);
    if (newWB + newWC > 1.0) { Real s = 1.0 / (newWB + newWC); newWB *= s; newWC *= s; }

    triIdx = bestTri;
    local = sofa::type::Vec3(newWB, newWC, 0);
}

template<class DataTypes>
void SmoothSlidingForceActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    unsigned int nbPoints = this->m_activeTriangles.size();
    if (!this->d_topology.get() || !this->m_state) return;
    const auto& triangles = this->d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = this->m_state->readPositions();
    WriteAccessor<Data<sofa::type::vector<sofa::type::Vec3>>> currentForces = this->d_currentForces;
    Real damping = this->d_stepDamping.getValue();
    // Max step in barycentric units (same conversion as updateLimit)
    Real maxStep_bary = (m_meanEdgeLength > 1e-12)
                        ? this->d_maxStepSize.getValue() / m_meanEdgeLength
                        : this->d_maxStepSize.getValue();

    for(unsigned int i=0; i<nbPoints; i++) {
        Real factor = this->d_jacobianScaleFactor.getValue();
        Real Fx = lambda[i*5 + 0] * factor;
        Real Fy = lambda[i*5 + 1] * factor;
        Real Fz = lambda[i*5 + 2] * factor;
        Real dwB = lambda[i*5 + 3] * factor;
        Real dwC = lambda[i*5 + 4] * factor;
        if (std::isnan(Fx + Fy + Fz + dwB + dwC)) continue;

        // Update force
        currentForces[i] = sofa::type::Vec3(Fx, Fy, Fz);

        // EMA: update smooth force direction used by next Jacobian build
        Real momentum = this->d_dirMomentum.getValue();
        if (momentum > 0.0 && this->m_smoothForces.size() > i)
            this->m_smoothForces[i] = this->m_smoothForces[i] * momentum
                                      + currentForces[i] * (1.0 - momentum);
        else if (this->m_smoothForces.size() > i)
            this->m_smoothForces[i] = currentForces[i];

        // Warm-start for next QP
        this->m_lambdaInit[i*5 + 0] = currentForces[i][0];
        this->m_lambdaInit[i*5 + 1] = currentForces[i][1];
        this->m_lambdaInit[i*5 + 2] = currentForces[i][2];
        this->m_lambdaInit[i*5 + 3] = 0.0;
        this->m_lambdaInit[i*5 + 4] = 0.0;

        // Sliding momentum: EMA of QP slide outputs.
        // Consistent signals accumulate; noisy signals cancel out.
        Real slideMom = this->d_slideMomentum.getValue();
        if (slideMom > 0.0) {
            m_slideMomentumB[i] = slideMom * m_slideMomentumB[i] + (1.0 - slideMom) * dwB;
            m_slideMomentumC[i] = slideMom * m_slideMomentumC[i] + (1.0 - slideMom) * dwC;
            dwB = m_slideMomentumB[i];
            dwC = m_slideMomentumC[i];
        }

        // Apply damping
        dwB *= damping;
        dwC *= damping;

        // Clamp step magnitude in barycentric space
        Real stepMag = std::sqrt(dwB*dwB + dwC*dwC);
        if (stepMag > maxStep_bary) {
            Real s = maxStep_bary / stepMag;
            dwB *= s; dwC *= s;
        }

        // Update barycentric coordinates
        this->m_activeLocalCoords[i][0] += dwB;
        this->m_activeLocalCoords[i][1] += dwC;

        // OOB check: if any weight < 0, project back onto mesh
        Real wB = this->m_activeLocalCoords[i][0];
        Real wC = this->m_activeLocalCoords[i][1];
        Real wA = 1.0 - wB - wC;
        if (wA < 0 || wB < 0 || wC < 0) {
            this->projectToMesh(this->m_activeTriangles[i], this->m_activeLocalCoords[i]);
            // Reset sliding momentum — barycentric frame changed
            if (slideMom > 0.0) {
                m_slideMomentumB[i] = 0.0;
                m_slideMomentumC[i] = 0.0;
            }
        }
    }
    this->d_triangleIndices.setValue(this->m_activeTriangles);
    this->d_localCoords.setValue(this->m_activeLocalCoords);

    // Compute world-space contact locations from barycentric
    sofa::type::vector<sofa::type::Vec3> currentLocations;
    currentLocations.resize(nbPoints);
    for(unsigned int i=0; i<nbPoints; i++) {
        unsigned int triIdx = this->m_activeTriangles[i];
        if(triIdx < triangles.size()) {
            const Triangle& t = triangles[triIdx];
            Real wB2 = this->m_activeLocalCoords[i][0];
            Real wC2 = this->m_activeLocalCoords[i][1];
            Real wA2 = 1.0 - wB2 - wC2;
            currentLocations[i] = sofa::type::Vec3(pos[t[0]]) * wA2
                                 + sofa::type::Vec3(pos[t[1]]) * wB2
                                 + sofa::type::Vec3(pos[t[2]]) * wC2;
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
        unsigned int triIdx = this->m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;
        const Triangle& t = triangles[triIdx];
        vparams->drawTool()->drawTriangle(pos[t[0]], pos[t[1]], pos[t[2]], sofa::type::Vec3(0,1,0), sofa::type::RGBAColor::yellow());

        // Contact point from barycentric
        Real wB = this->m_activeLocalCoords[i][0];
        Real wC = this->m_activeLocalCoords[i][1];
        Real wA = 1.0 - wB - wC;
        Coord P = sofa::type::Vec3(pos[t[0]]) * wA
                + sofa::type::Vec3(pos[t[1]]) * wB
                + sofa::type::Vec3(pos[t[2]]) * wC;

        sofa::type::Vec3 f = this->d_currentForces.getValue().size() > i
                           ? this->d_currentForces.getValue()[i]
                           : sofa::type::Vec3(0, 0, 0);
        if (f.norm2() < 1e-12) continue;
        sofa::type::Vec3 dir = f / f.norm();
        vparams->drawTool()->drawArrow(
            P - dir * log(f.norm()+1) * this->d_visuScale.getValue(),
            P,
            log(f.norm()+1) * this->d_visuScale.getValue() / 20.0,
            sofa::type::RGBAColor::red());
    }
}

} // namespace
