#pragma once

#include <unordered_set>

#include <SoftRobots.Inverse/component/constraint/SlidingForceActuator.h>
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
SlidingForceActuator<DataTypes>::SlidingForceActuator(MechanicalState* object)
    : Inherit1(object)
    , d_triangleIndices(initData(&d_triangleIndices, "triangleIndices", "Indices of active triangles"))
    , d_localCoords(initData(&d_localCoords, "localCoords", "Local Cartesian coords (U, V, 0) in tangent plane for each point."))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max normal force"))
    , d_minForce(initData(&d_minForce, "minForce", "Min normal force"))
    , d_initForce(initData(&d_initForce, Real(0.0), "initForce", "Initial force guess"))
    , d_maxForceStep(initData(&d_maxForceStep, Real(0.0), "maxForceStep", "Max change in force magnitude per iteration (0 = no limit)"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.1), "maxStepSize", "Trust region for sliding (Cartesian step limit in tangent plane)"))
    , d_stepDamping(initData(&d_stepDamping, Real(0.5), "stepDamping", "Damping factor for sliding step (0.1). Lower reduces jitter."))
    , d_epsilon(initData(&d_epsilon, Real(1e-3), "epsilon", 
                "Regularization for the constraint. Use this value to prioritize the constraint. 0 means no limitation on the energy transfered by this actuator. Default is 1e-3."))
    , d_epsilonForce(initData(&d_epsilonForce, Real(1e-3), "epsilonForce",
                           "Use this value to prioritize the constraint. 0 means no limitation on the energy transfered by this actuator. Default is 1e-3."))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding",
                           "Use this value to prioritize the sliding constraint. Default is 1e-3."))
    , d_ridgeForce(initData(&d_ridgeForce, Real(1e-12), "ridgeForce", 
                        "Ridge for force variable to improve convergence when the optimal force is close to zero. Default is 1e-12."))
    , d_ridgeSliding(initData(&d_ridgeSliding, Real(1e-12), "ridgeSliding", 
                        "Ridge for sliding variable to improve convergence when the optimal sliding is close to zero. Default is 1e-12."))
    , d_currentForces(initData(&d_currentForces, "currentForces", "Current forces applied"))
    , d_currentLocation(initData(&d_currentLocation, "currentLocation", "Current force locations in world coordinates"))
    , d_showForce(initData(&d_showForce, false, "showForce", "Visualize forces"))
    , d_visuScale(initData(&d_visuScale, Real(0.1), "visuScale", "Scale for visualization"))
    , d_topology(initLink("topology", "Mesh topology"))
{
    d_showForce.setGroup("Visualization");
    d_visuScale.setGroup("Visualization");
}

template<class DataTypes>
SlidingForceActuator<DataTypes>::~SlidingForceActuator()
{
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::init()
{
    Inherit1::init();
    
    if (!d_topology.get()) {
        msg_warning() << "No topology linked. Trying to find one in context.";
        d_topology.set(this->getContext()->getMeshTopology());
    }

    initData();
    updateLimit();
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::reinit()
{
    initData();
    updateLimit();
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::initData()
{
    unsigned int nbPoints = d_triangleIndices.getValue().size();
    m_dim = nbPoints * 5;
    m_nbLines = m_dim;

    m_activeTriangles = d_triangleIndices.getValue();
    if(d_localCoords.getValue().size() == nbPoints)
    {
        m_activeLocalCoords = d_localCoords.getValue();
    }
    else
    {
        m_activeLocalCoords.assign(nbPoints, sofa::type::Vec3(0,0,0));
        const auto& triangles = (d_topology.get()) ? d_topology.get()->getTriangles() : sofa::type::vector<Triangle>();
        if (m_state && !triangles.empty()) {
             ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
             for(unsigned int i=0; i<nbPoints; i++) {
                 unsigned int triIdx = m_activeTriangles[i];
                 if(triIdx < triangles.size()) {
                     const Triangle& t = triangles[triIdx];
                     Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
                     Coord P = (A+B+C)/3.0;
                     Deriv v1 = B-A;
                     sofa::type::Vec3 e1 = v1; e1.normalize();
                     m_activeLocalCoords[i][0] = (P-A)*e1;
                     sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                     sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
                     m_activeLocalCoords[i][1] = (P-A)*e2;
                 }
             }
        }
        if (!d_localCoords.getValue().empty())
            msg_warning() << "SlidingForceActuator: 'localCoords' data size mismatch. Using triangle centers.";
    }

    if(d_epsilon.isSet())
    {
        m_hasEpsilon = true;
        m_epsilon = d_epsilon.getValue();
    }

    unsigned int dim = nbPoints * 5;
    m_lambdaInit.assign(dim, 0.0);
    m_lambdaMax.resize(dim);
    m_lambdaMin.resize(dim);

    if(d_initForce.isSet())
    {
        m_hasLambdaInit = true;
        Real f0 = d_initForce.getValue();
        const auto& triangles = (d_topology.get()) ? d_topology.get()->getTriangles() : sofa::type::vector<Triangle>();
        ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
        for (unsigned int i=0; i<nbPoints; i++) {
            unsigned int triIdx = m_activeTriangles[i];
            if(triIdx >= triangles.size()) continue;
            const Triangle& tri = triangles[triIdx];
            const Coord& A = pos[tri[0]];
            const Coord& B = pos[tri[1]];
            const Coord& C = pos[tri[2]];
            sofa::type::Vec3 n = sofa::type::cross(B-A, C-A);
            n.normalize();

            m_lambdaInit[i*5 + 0] = n[0] * f0; 
            m_lambdaInit[i*5 + 1] = n[1] * f0;
            m_lambdaInit[i*5 + 2] = n[2] * f0; 
            m_lambdaInit[i*5 + 3] = 0.0;
            m_lambdaInit[i*5 + 4] = 0.0; 
        }
    }

    if(d_maxForce.isSet())
        m_hasLambdaMax = true;

    if(d_minForce.isSet())
        m_hasLambdaMin = true;

    if (d_maxForceStep.isSet() && d_maxForceStep.getValue() > 0.0)
    {
        m_hasLambdaMax = true;
        m_hasLambdaMin = true;
    }
    
    // Initialize forces using normal direction and initForce magnitude
    sofa::type::vector<sofa::type::Vec3> currentForces;
    currentForces.resize(nbPoints);
    if (d_topology.get() && m_state) {
        const auto& triangles = d_topology.get()->getTriangles();
        ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
        Real fMag = d_initForce.getValue();
        if (fMag == 0.0) fMag = 1e-3; // Fallback to avoid singular Jacobian

        for(unsigned int i=0; i<nbPoints; i++) {
             unsigned int triIdx = m_activeTriangles[i];
             if(triIdx < triangles.size()) {
                 const Triangle& t = triangles[triIdx];
                 const Coord& A = pos[t[0]];
                 const Coord& B = pos[t[1]];
                 const Coord& C = pos[t[2]];
                 
                 sofa::type::Vec3 n = sofa::type::cross(B-A, C-A);
                 n.normalize();
                 currentForces[i] = n * fMag;
             } else {
                 currentForces[i] = sofa::type::Vec3(0,0,0);
             }
        }
    } else {
         sofa::type::Vec3 initF(0,0,0);
         std::fill(currentForces.begin(), currentForces.end(), initF);
    }
    d_currentForces.setValue(currentForces);

    // Update current location
    if (m_state && d_topology.get()) {
        const auto& triangles = d_topology.get()->getTriangles();
        ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
        sofa::type::vector<sofa::type::Vec3> currentLocations;
        currentLocations.resize(nbPoints);
        for(unsigned int i=0; i<nbPoints; i++) {
            unsigned int triIdx = m_activeTriangles[i];
            if(triIdx < triangles.size()) {
                const Triangle& t = triangles[triIdx];
                Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
                Deriv v1 = B-A;
                sofa::type::Vec3 e1 = v1; e1.normalize();
                sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
                currentLocations[i] = A + e1 * m_activeLocalCoords[i][0] + e2 * m_activeLocalCoords[i][1];
            }
        }
        d_currentLocation.setValue(currentLocations);
    }

    updateLimit();
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::updateLimit()
{
    Real maxF = d_maxForce.isSet() ? d_maxForce.getValue() : 1e20;
    Real minF = d_minForce.isSet() ? d_minForce.getValue() : -1e20;
    Real step = d_maxStepSize.getValue();
    Real maxForceStep = d_maxForceStep.getValue();
    
    const auto& triangles = (d_topology.get()) ? d_topology.get()->getTriangles() : sofa::type::vector<Triangle>();

    for(unsigned int i=0; i<d_currentForces.getValue().size(); i++) {
        // Compute Scaling Factor
        sofa::type::Vec3 currentForce = d_currentForces.getValue()[i];
        
        if (currentForce.norm2() < 1e-12 && m_state && d_topology.get()) {
             // Virtual Force Logic
             ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
             unsigned int triIdx = m_activeTriangles[i];
             if(triIdx < triangles.size()) {
                 const Triangle& t = triangles[triIdx];
                 const Coord& A = pos[t[0]];
                 const Coord& B = pos[t[1]];
                 const Coord& C = pos[t[2]];
                 sofa::type::Vec3 n = sofa::type::cross(B-A, C-A);
                 n.normalize();
                 Real fScale = 1.0;
                 if (d_initForce.isSet() && d_initForce.getValue() > 1e-9) fScale = d_initForce.getValue();
                 currentForce = n * fScale;
             }
        }                                                                                                                
        Real jacobianScale = currentForce.norm();                                                                 
        if (jacobianScale < 1e-9) jacobianScale = 1.0;   

        // Force bounds (Indices 0, 1, 2)
        if (maxForceStep > 0.0)
        {
            m_lambdaMin[i*5 + 0] = std::max(minF, currentForce[0] - maxForceStep);
            m_lambdaMax[i*5 + 0] = std::min(maxF, currentForce[0] + maxForceStep);
            m_lambdaMin[i*5 + 1] = std::max(minF, currentForce[1] - maxForceStep);
            m_lambdaMax[i*5 + 1] = std::min(maxF, currentForce[1] + maxForceStep);
            m_lambdaMin[i*5 + 2] = std::max(minF, currentForce[2] - maxForceStep);
            m_lambdaMax[i*5 + 2] = std::min(maxF, currentForce[2] + maxForceStep);
        }
        else
        {
            m_lambdaMin[i*5 + 0] = minF;
            m_lambdaMax[i*5 + 0] = maxF;
            m_lambdaMin[i*5 + 1] = minF;
            m_lambdaMax[i*5 + 1] = maxF;
            m_lambdaMin[i*5 + 2] = minF;
            m_lambdaMax[i*5 + 2] = maxF;
        }
        
        // Sliding bounds (Indices 3, 4) - SCALED Bounds for Cartesian dU, dV
        Real scaledStep = step  ; //* jacobianScale;
        
        m_lambdaMin[i*5 + 3] = -scaledStep;
        m_lambdaMax[i*5 + 3] = scaledStep;
        
        m_lambdaMin[i*5 + 4] = -scaledStep;
        m_lambdaMax[i*5 + 4] = scaledStep;
    }
}


template<class DataTypes>
void SlidingForceActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                          DataMatrixDeriv &cMatrix,
                                                          unsigned int &cIndex,
                                                          const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    
    d_constraintIndex.setValue(cIndex);
    unsigned int startConstraintIndex = cIndex;
    
    if (!d_topology.get()) return;
    
    const auto& triangles = d_topology.get()->getTriangles();
    const VecCoord& pos = x.getValue();
    
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    
    for(unsigned int i=0; i<m_activeTriangles.size(); i++)
    {
        unsigned int triIdx = m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;

        const Triangle& tri = triangles[triIdx];
        sofa::type::Vec3 local = m_activeLocalCoords[i];
        
        const Coord& A = pos[tri[0]];
        const Coord& B = pos[tri[1]];
        const Coord& C = pos[tri[2]];
        Deriv v1 = B - A;
        Deriv v2 = C - A;
        sofa::type::Vec3 nBasis = sofa::type::cross(v1, v2);
        Real area2 = nBasis.norm();
        if (area2 > 1e-12) nBasis /= area2;

        sofa::type::Vec3 e1 = v1;
        e1.normalize();
        sofa::type::Vec3 e2 = sofa::type::cross(nBasis, e1);

        // Compute weights from Cartesian coords
        // P = A + U*e1 + V*e2
        // We need wA, wB, wC such that P = wA*A + wB*B + wC*C
        // P-A = wB(B-A) + wC(C-A) = wB*v1 + wC*v2
        
        Real v1x = v1 * e1;
        Real v2x = v2 * e1;
        Real v2y = v2 * e2;
        Real det = v1x * v2y;
        if (std::abs(det) < 1e-12) det = 1.0;

        Real weightC = local[1] / v2y;
        Real weightB = (local[0] - weightC * v2x) / v1x;
        Real weightA = 1.0 - weightB - weightC;

        // Current Force Vector
        sofa::type::Vec3 currentForce = d_currentForces.getValue()[i];
        sofa::type::Vec3 gradientForce = currentForce;
        
        // Handle vanishing gradients when force is zero.
        if (gradientForce.norm2() < 1e-12) {
            Real scale = 1.0;
            if (d_initForce.isSet() && d_initForce.getValue() > 1e-9) scale = d_initForce.getValue();
            gradientForce = nBasis * scale;
        }
        
        // SCALING: Normalize the gradient to avoid ill-conditioning.
        Real jacobianScale = gradientForce.norm();
        if (jacobianScale < 1e-9) jacobianScale = 1.0;
        sofa::type::Vec3 scaledGradient = gradientForce / jacobianScale;

        // --- Rows 0, 1, 2: Force Components (Fx, Fy, Fz) ---
        // Row 0: Fx
        MatrixDerivRowIterator rowFx = matrix.writeLine(cIndex++);
        rowFx.addCol(tri[0], Deriv(weightA, 0, 0));
        rowFx.addCol(tri[1], Deriv(weightB, 0, 0));
        rowFx.addCol(tri[2], Deriv(weightC, 0, 0));
        
        // Row 1: Fy
        MatrixDerivRowIterator rowFy = matrix.writeLine(cIndex++);
        rowFy.addCol(tri[0], Deriv(0, weightA, 0));
        rowFy.addCol(tri[1], Deriv(0, weightB, 0));
        rowFy.addCol(tri[2], Deriv(0, weightC, 0));
        
        // Row 2: Fz
        MatrixDerivRowIterator rowFz = matrix.writeLine(cIndex++);
        rowFz.addCol(tri[0], Deriv(0, 0, weightA));
        rowFz.addCol(tri[1], Deriv(0, 0, weightB));
        rowFz.addCol(tri[2], Deriv(0, 0, weightC));
        
        // --- Rows 3, 4: Sliding (dU, dV) in Cartesian Tangent Plane ---
        Real du_dU = 1.0 / v1x;
        Real du_dV = -v2x / det;
        Real dv_dU = 0.0;
        Real dv_dV = 1.0 / v2y;

        // Row 3: Sliding U (dU)
        MatrixDerivRowIterator rowSlideU = matrix.writeLine(cIndex++);
        rowSlideU.addCol(tri[0], (-(du_dU + dv_dU)) * scaledGradient);
        rowSlideU.addCol(tri[1], du_dU * scaledGradient);
        rowSlideU.addCol(tri[2], dv_dU * scaledGradient);
        
        // Row 4: Sliding V (dV)
        MatrixDerivRowIterator rowSlideV = matrix.writeLine(cIndex++);
        rowSlideV.addCol(tri[0], (-(du_dV + dv_dV)) * scaledGradient);
        rowSlideV.addCol(tri[1], du_dV * scaledGradient);
        rowSlideV.addCol(tri[2], dv_dV * scaledGradient);
    }
    
    cMatrix.endEdit();
    m_nbLines = cIndex - startConstraintIndex;

}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                           BaseVector *resV,
                                                           const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(Jdx);
    // Target is zero (minimization of variables)
    // No violation
    unsigned int dim = m_activeTriangles.size() * 5;
    const auto& constraintId = sofa::helper::getReadAccessor(d_constraintIndex);
    for(unsigned int i=0; i<dim; i++)
        resV->set(constraintId + i, 0.);
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::projectToMesh(unsigned int& triIdx, sofa::type::Vec3& local)
{
    if (!d_topology.get() || !m_state) return;
    
    const auto& triangles = d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
    
    // 1. Reconstruct 3D position from current (possibly illegal) local coords
    if (triIdx >= triangles.size()) return;
    const Triangle& t = triangles[triIdx];
    
    Coord A0 = pos[t[0]];
    Coord B0 = pos[t[1]];
    Coord C0 = pos[t[2]];
    
    Deriv v1_0 = B0 - A0;
    sofa::type::Vec3 e1_0 = v1_0; e1_0.normalize();
    sofa::type::Vec3 n_0 = sofa::type::cross(B0-A0, C0-A0); n_0.normalize();
    sofa::type::Vec3 e2_0 = sofa::type::cross(n_0, e1_0);

    Coord candidatePos = A0 + e1_0 * local[0] + e2_0 * local[1];
    
    // 2. Find closest triangle
    Real minDist = 1e99;
    int bestTri = -1;
    sofa::type::Vec3 bestLocal;
    
    for(unsigned int i=0; i<triangles.size(); i++) {
        sofa::type::Vec3 close;
        const Triangle& tri = triangles[i];
        bool ok = sofa::geometry::proximity::computeClosestPointOnTriangleToPoint(
            sofa::type::Vec3(pos[tri[0]]), 
            sofa::type::Vec3(pos[tri[1]]), 
            sofa::type::Vec3(pos[tri[2]]), 
            sofa::type::Vec3(candidatePos), 
            close
        );
        if (ok) {
            Real d = (Coord(close) - candidatePos).norm();
            if (d < minDist) {
                minDist = d;
                bestTri = i;
                
                // Compute new local coords for this triangle
                Coord A = pos[tri[0]];
                Coord B = pos[tri[1]];
                Coord C = pos[tri[2]];
                Deriv v1 = B - A;
                sofa::type::Vec3 e1 = v1; e1.normalize();
                sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                sofa::type::Vec3 e2 = sofa::type::cross(n, e1);

                bestLocal[0] = (Coord(close) - A) * e1;
                bestLocal[1] = (Coord(close) - A) * e2;
                bestLocal[2] = 0;
            }
        }
    }
    
    if (bestTri != -1) {
        triIdx = bestTri;
        local = bestLocal;
    }
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    SOFA_UNUSED(delta);

    // std::cout<<"SlidingForceActuator::storeResults - Lambda: ";
    // for (auto i: lambda)
    //     std::cout<<i<<", ";
    // std::cout<<std::endl;

    unsigned int n_triangles = m_activeTriangles.size();
    // unsigned int startId = d_constraintIndex.getValue();
    
    if (!d_topology.get() || !m_state) return;
    const auto& triangles = d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();

    
    sofa::helper::WriteAccessor< sofa::Data<sofa::type::vector<sofa::type::Vec3>> > currentForces = d_currentForces;
    Real maxStep = d_maxStepSize.getValue();
    Real damping = d_stepDamping.getValue();
    if (damping < 0.0) damping = 0.0;
    if (damping > 1.0) damping = 1.0;
    
    for(unsigned int i=0; i<n_triangles; i++) {
        Real Fx = lambda[ i*5 + 0];
        Real Fy = lambda[ i*5 + 1];
        Real Fz = lambda[ i*5 + 2];
        Real dU = lambda[ i*5 + 3];
        Real dV = lambda[ i*5 + 4];

        unsigned int triangleIdx = m_activeTriangles[i];
        const Triangle& tri = triangles[triangleIdx];

        sofa::type::Vec3 currentForce = d_currentForces.getValue()[i];
        sofa::type::Vec3 gradientForce = currentForce;
        
        // Handle vanishing gradients when force is zero.
        if (gradientForce.norm2() < 1e-12) {
            const Coord& A = pos[tri[0]];
            const Coord& B = pos[tri[1]];
            const Coord& C = pos[tri[2]];
            sofa::type::Vec3 n = sofa::type::cross(B-A, C-A);
            n.normalize();
            
            Real scale = 1.0;
            if (d_initForce.isSet() && d_initForce.getValue() > 1e-9) scale = d_initForce.getValue();
            
            gradientForce = n * scale;
        }
        Real jacobianScale = gradientForce.norm();
        if (jacobianScale < 1e-9) jacobianScale = 1.0;
        
        // DECODING: Recover physical step (Cartesian distance in tangent plane)
        // dU /= jacobianScale;
        // dV /= jacobianScale;

        // Safety check 1: Detect NaN/Inf
        if (std::isnan(Fx) || std::isnan(Fy) || std::isnan(Fz) || std::isnan(dU) || std::isnan(dV) ||
            std::isinf(Fx) || std::isinf(Fy) || std::isinf(Fz) || std::isinf(dU) || std::isinf(dV)) {
            msg_warning() << "SlidingForceActuator: Solver returned NaN/Inf! Skipping update for point " << i;
            continue; 
        }

        // Apply damping to reduce jitter
        dU *= damping;
        dV *= damping;

        // Safety check 2: Detect Garbage and Clamp
        if (std::abs(dU) > maxStep || std::abs(dV) > maxStep) {
             if (dU > maxStep) dU = maxStep;
             else if (dU < -maxStep) dU = -maxStep;
             
             if (dV > maxStep) dV = maxStep;
             else if (dV < -maxStep) dV = -maxStep;
        }

        currentForces[i] = sofa::type::Vec3(Fx, Fy, Fz);

        // Update Cartesian Local Coords
        m_activeLocalCoords[i][0] += dU;
        m_activeLocalCoords[i][1] += dV;

        // Check OOB and project. 
        // We can check OOB by converting to barycentric
        const Coord& A = pos[tri[0]];
        const Coord& B = pos[tri[1]];
        const Coord& C = pos[tri[2]];
        Deriv v1 = B - A;
        Deriv v2 = C - A;
        sofa::type::Vec3 nBasis = sofa::type::cross(v1, v2);
        Real area2 = nBasis.norm();
        if (area2 > 1e-12) nBasis /= area2;
        sofa::type::Vec3 e1 = v1; e1.normalize();
        sofa::type::Vec3 e2 = sofa::type::cross(nBasis, e1);

        Real v1x = v1 * e1;
        Real v2x = v2 * e1;
        Real v2y = v2 * e2;
        Real det = v1x * v2y;
        if (std::abs(det) < 1e-12) det = 1.0;

        Real weightC = m_activeLocalCoords[i][1] / v2y;
        Real weightB = (m_activeLocalCoords[i][0] - weightC * v2x) / v1x;
        Real weightA = 1.0 - weightB - weightC;

        bool oob = (weightA < 0 || weightB < 0 || weightC < 0);

        if (oob) {
            projectToMesh(m_activeTriangles[i], m_activeLocalCoords[i]);
        }
    }
    
    // Update Data for next step visualization/read
    d_triangleIndices.setValue(m_activeTriangles);
    d_localCoords.setValue(m_activeLocalCoords);

    // Update current location
    {
        const auto& triangles = d_topology.get()->getTriangles();
        ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
        sofa::type::vector<sofa::type::Vec3> currentLocations;
        currentLocations.resize(m_activeTriangles.size());
        for(unsigned int i=0; i<m_activeTriangles.size(); i++) {
            unsigned int triIdx = m_activeTriangles[i];
            if(triIdx < triangles.size()) {
                const Triangle& t = triangles[triIdx];
                Coord A = pos[t[0]]; Coord B = pos[t[1]]; Coord C = pos[t[2]];
                Deriv v1 = B-A;
                sofa::type::Vec3 e1 = v1; e1.normalize();
                sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
                sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
                currentLocations[i] = A + e1 * m_activeLocalCoords[i][0] + e2 * m_activeLocalCoords[i][1];
            }
        }
        d_currentLocation.setValue(currentLocations);
    }

    // std::cout << "Current Location "<<d_currentLocation.getValue()[0] << std::endl;
    
    updateLimit(); // Important for bounds!
    
    // Base class storeResults handles delta
    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields() || !d_showForce.getValue())
        return;

    if (!d_topology.get() || !m_state) return;
    
    vparams->drawTool()->setLightingEnabled(true);
    
    const auto& triangles = d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
    
    // Highlight active triangles
    for(unsigned int i=0; i<m_activeTriangles.size(); i++) {
        unsigned int triIdx = m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;
        
        const Triangle& t = triangles[triIdx];
        vparams->drawTool()->drawTriangle(pos[t[0]], pos[t[1]], pos[t[2]], sofa::type::Vec3(0,1,0), sofa::type::RGBAColor::yellow());
    }

    for(unsigned int i=0; i<m_activeTriangles.size(); i++) {
        unsigned int triIdx = m_activeTriangles[i];
        if (triIdx >= triangles.size()) continue;
        
        const Triangle& t = triangles[triIdx];
        Coord A = pos[t[0]];
        Coord B = pos[t[1]];
        Coord C = pos[t[2]];
        
        sofa::type::Vec3 local = m_activeLocalCoords[i];
        Deriv v1 = B - A;
        sofa::type::Vec3 e1 = v1; e1.normalize();
        sofa::type::Vec3 n = sofa::type::cross(B-A, C-A); n.normalize();
        sofa::type::Vec3 e2 = sofa::type::cross(n, e1);
        
        Coord P = A + e1 * local[0] + e2 * local[1];
        
        sofa::type::Vec3 f = d_currentForces.getValue()[i];
        if (f.norm2() < 1e-12) continue;
        sofa::type::Vec3 dir = f/f.norm();
        
        vparams->drawTool()->drawArrow(P - dir * log(f.norm()+1)*d_visuScale.getValue(), P, 
                                        log(f.norm()+1)*d_visuScale.getValue()/20.0, 
                                        sofa::type::RGBAColor::red());
    }
}

} // namespace