#pragma once

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
    , d_barycentric(initData(&d_barycentric, "barycentric", "Barycentric coords (u, v, w) for each point. w is ignored (1-u-v)."))
    , d_maxForce(initData(&d_maxForce, "maxForce", "Max normal force"))
    , d_minForce(initData(&d_minForce, "minForce", "Min normal force"))
    , d_initForce(initData(&d_initForce, Real(0.0), "initForce", "Initial force guess"))
    , d_maxStepSize(initData(&d_maxStepSize, Real(0.1), "maxStepSize", "Trust region for sliding (barycentric step limit)"))
    , d_epsilon(initData(&d_epsilon, Real(1e-3), "epsilon",
                           "Use this value to prioritize the constraint. 0 means no limitation on the energy transfered by this actuator. Default is 1e-3."))
    , d_epsilonSliding(initData(&d_epsilonSliding, Real(1e-3), "epsilonSliding",
                           "Use this value to prioritize the sliding constraint. Default is 1e-3."))
    , d_currentForces(initData(&d_currentForces, "currentForces", "Current forces applied"))
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

    m_activeTriangles = d_triangleIndices.getValue();
    if(d_barycentric.getValue().size() == nbPoints)
    {
        m_activeBarycentric = d_barycentric.getValue();
    }
    else
    {
        m_activeBarycentric.assign(nbPoints, sofa::type::Vec3(1.0/3.0, 1.0/3.0, 1.0/3.0));
        if (!d_barycentric.getValue().empty())
            msg_warning() << "SlidingForceActuator: 'barycentric' data size mismatch. Using default center (1/3, 1/3, 1/3).";
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
        for(unsigned int i=0; i<nbPoints; i++) {
            m_lambdaInit[i*5 + 0] = f0;
            m_lambdaInit[i*5 + 1] = f0;
            m_lambdaInit[i*5 + 2] = f0;
        }
    }

    if(d_maxForce.isSet())
        m_hasLambdaMax = true;

    if(d_minForce.isSet())
        m_hasLambdaMin = true;
    
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

    updateLimit();
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::updateLimit()
{
    Real maxF = d_maxForce.isSet() ? d_maxForce.getValue() : 1e20;
    Real minF = d_minForce.isSet() ? d_minForce.getValue() : -1e20;
    Real step = d_maxStepSize.getValue();
    
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
        m_lambdaMin[i*5 + 0] = minF;
        m_lambdaMax[i*5 + 0] = maxF;
        m_lambdaMin[i*5 + 1] = minF;
        m_lambdaMax[i*5 + 1] = maxF;
        m_lambdaMin[i*5 + 2] = minF;
        m_lambdaMax[i*5 + 2] = maxF;
        
        // Sliding bounds (Indices 3, 4) - SCALED Bounds for Scaled u, v
        // The Jacobian is scaled by 1/scale. The variable lambda is scaled by scale.
        // So the limit must be step * scale.
        Real scaledStep = step * jacobianScale;
        
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
        sofa::type::Vec3 bary = m_activeBarycentric[i];
    
        // Barycentric weights for A, B, C
        // P = wA*A + wB*B + wC*C
        Real weightA = 1.0 - bary[0] - bary[1];
        Real weightB = bary[0];
        Real weightC = bary[1];
        
        // Current Force Vector
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
        
        // SCALING: Normalize the gradient to avoid ill-conditioning.
        // The solver prefers entries ~ O(1). Current Force can be ~10,000.
        Real jacobianScale = gradientForce.norm();
        if (jacobianScale < 1e-9) jacobianScale = 1.0;
        sofa::type::Vec3 scaledGradient = gradientForce / jacobianScale;

        // --- Rows 0, 1, 2: Force Components (Fx, Fy, Fz) ---
        // Force F applied at P distributes to nodes A, B, C with weights wA, wB, wC.
        
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
        
        // --- Rows 3, 4: Sliding (dU, dV) ---
        // Effect of changing u, v on the nodal forces.
        // Use SCALED Gradient to keep Matrix Condition Number low.

        const Real virtualStiffness = 1e-4;
        
        // Row 3: Sliding U (dU)
        MatrixDerivRowIterator rowSlideU = matrix.writeLine(cIndex++);
        rowSlideU.addCol(tri[0], -scaledGradient);
        rowSlideU.addCol(tri[1],  scaledGradient);
        rowSlideU.addCol(tri[2],  Deriv(0,0,0));
        // 2. The Virtual Spring (Effect of geometry/position)
        // This links dU directly to the relative positions of A and B
        rowSlideU.addCol(tri[0], Deriv(-virtualStiffness, 0, 0)); 
        rowSlideU.addCol(tri[1], Deriv( virtualStiffness, 0, 0));
        
        // Row 4: Sliding V (dV)
        MatrixDerivRowIterator rowSlideV = matrix.writeLine(cIndex++);
        rowSlideV.addCol(tri[0], -scaledGradient);
        rowSlideV.addCol(tri[1],  Deriv(0,0,0));
        rowSlideV.addCol(tri[2],  scaledGradient);

        // 2. The Virtual Spring
        rowSlideV.addCol(tri[0], Deriv(0, -virtualStiffness, 0)); 
        rowSlideV.addCol(tri[2], Deriv(0,  virtualStiffness, 0));
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
void SlidingForceActuator<DataTypes>::projectToMesh(unsigned int& triIdx, sofa::type::Vec3& bary)
{
    // If barycentric coords are outside [0,1], we crossed an edge.
    
    if (!d_topology.get() || !m_state) return;
    
    const auto& triangles = d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();
    
    // 1. Compute current candidate 3D position
    if (triIdx >= triangles.size()) return;
    const Triangle& t = triangles[triIdx];
    
    // Reconstruct 3D pos from "illegal" barycentric coords
    // P = A + u(B-A) + v(C-A)
    Coord A = pos[t[0]];
    Coord B = pos[t[1]];
    Coord C = pos[t[2]];
    
    Coord candidatePos = A + (B-A)*bary[0] + (C-A)*bary[1];
    
    // 2. Find closest triangle
    Real minDist = 1e99;
    int bestTri = -1;
    sofa::type::Vec3 bestBary;
    
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
                // Compute bary coords for this triangle
                Coord p0 = pos[tri[0]]; Coord p1 = pos[tri[1]]; Coord p2 = pos[tri[2]];
                Deriv v0 = p1 - p0; Deriv v1 = p2 - p0; Deriv v2 = Coord(close) - p0;
                Real d00 = v0*v0; Real d01 = v0*v1; Real d11 = v1*v1; Real d20 = v2*v0; Real d21 = v2*v1;
                Real denom = d00*d11 - d01*d01;
                if (denom > 1e-12) {
                    Real v = (d11*d20 - d01*d21) / denom;
                    Real w = (d00*d21 - d01*d20) / denom;
                    bestBary = sofa::type::Vec3(v, w, 1.0-v-w);
                } else {
                    bestBary = sofa::type::Vec3(0,0,1);
                }
            }
        }
    }
    
    if (bestTri != -1) {
        triIdx = bestTri;
        bary = bestBary;
    }
}

template<class DataTypes>
void SlidingForceActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    SOFA_UNUSED(delta);

    std::cout<<"SlidingForceActuator::storeResults - Lambda: ";
    for (auto i: lambda)
        std::cout<<i<<", ";
    std::cout<<std::endl;

    unsigned int n_triangles = m_activeTriangles.size();
    // unsigned int startId = d_constraintIndex.getValue();
    
    if (!d_topology.get() || !m_state) return;
    const auto& triangles = d_topology.get()->getTriangles();
    ReadAccessor<Data<VecCoord>> pos = m_state->readPositions();

    
    sofa::helper::WriteAccessor< sofa::Data<sofa::type::vector<sofa::type::Vec3>> > currentForces = d_currentForces;
    Real maxStep = d_maxStepSize.getValue();
    
    for(unsigned int i=0; i<n_triangles; i++) {
        Real Fx = lambda[ i*5 + 0];
        Real Fy = lambda[ i*5 + 1];
        Real Fz = lambda[ i*5 + 2];
        Real dU = lambda[ i*5 + 3];
        Real dV = lambda[ i*5 + 4];

        // std::cout << "startID: " << startId << " i: " << i << std::endl;

        std::cout <<"Triangle " << m_activeTriangles[i] << " Lambda Force: " << Fx << "," << Fy << "," << Fz 
                  << " Sliding Step: " << dU << "," << dV << std::endl;
        
        // Recompute Scaling Factor to decode dU/dV
        sofa::type::Vec3 gradientForce = currentForces[i];
        // Handle Virtual Force reconstruction if needed (simplified check)
        if (gradientForce.norm2() < 1e-12) {
            unsigned int triIdx = m_activeTriangles[i];
            if(triIdx < triangles.size()) {
                const Triangle& t = triangles[triIdx];
                const Coord& A = pos[t[0]];
                const Coord& B = pos[t[1]];
                const Coord& C = pos[t[2]];
                sofa::type::Vec3 n = sofa::type::cross(B-A, C-A);
                n.normalize();
                Real scale = 1.0;
                if (d_initForce.isSet() && d_initForce.getValue() > 1e-9) scale = d_initForce.getValue();
                gradientForce = n * scale;
            }
        }
        Real jacobianScale = gradientForce.norm();
        if (jacobianScale < 1e-9) jacobianScale = 1.0;
        
        // DECODING: Recover physical step
        // Since we scaled the Jacobian by (1/scale), the lambda returned is (scale * step).
        dU /= jacobianScale;
        dV /= jacobianScale;

        // Safety check 1: Detect NaN/Inf
        if (std::isnan(Fx) || std::isnan(Fy) || std::isnan(Fz) || std::isnan(dU) || std::isnan(dV) ||
            std::isinf(Fx) || std::isinf(Fy) || std::isinf(Fz) || std::isinf(dU) || std::isinf(dV)) {
            msg_warning() << "SlidingForceActuator: Solver returned NaN/Inf! Skipping update for point " << i;
            continue; 
        }

        // Safety check 2: Detect Garbage and Clamp
        // We check the physical step against the limit
        if (std::abs(dU) > maxStep || std::abs(dV) > maxStep) {
             if (std::abs(dU) > maxStep * 2.0 || std::abs(dV) > maxStep * 2.0) {
                 msg_warning() << "SlidingForceActuator: Solver returned step size " << dU << ", " << dV 
                               << " exceeding limit " << maxStep << ". Clamping.";
             }
             if (dU > maxStep) dU = maxStep;
             else if (dU < -maxStep) dU = -maxStep;
             
             if (dV > maxStep) dV = maxStep;
             else if (dV < -maxStep) dV = -maxStep;
        }

        currentForces[i] = sofa::type::Vec3(Fx, Fy, Fz);
        
        // Update Barycentric
        m_activeBarycentric[i][0] += dU;
        m_activeBarycentric[i][1] += dV;

        std::cout << "Current Force: " << currentForces[i][0] << "," << currentForces[i][1] << "," << currentForces[i][2]
                  <<" dU: " << dU << " dV: " << dV 
                  << " New Barycentric: " << m_activeBarycentric[i][0] << "," << m_activeBarycentric[i][1] << std::endl;
        
        // Check bounds [0, 1] and sum <= 1
        bool oob = (m_activeBarycentric[i][0] < 0 || m_activeBarycentric[i][1] < 0 || (m_activeBarycentric[i][0]+m_activeBarycentric[i][1] > 1));

        
        if (oob) {
            projectToMesh(m_activeTriangles[i], m_activeBarycentric[i]);
        }
    }
    
    // Update Data for next step visualization/read
    d_triangleIndices.setValue(m_activeTriangles);
    d_barycentric.setValue(m_activeBarycentric);
    
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
        
        Real u = m_activeBarycentric[i][0];
        Real v = m_activeBarycentric[i][1];
        
        Coord P = A + (B-A)*u + (C-A)*v;
        
        sofa::type::Vec3 f = d_currentForces.getValue()[i];
        sofa::type::Vec3 dir = f/f.norm();
        
        // vparams->drawTool()->drawArrow(P, P + n * f * d_visuScale.getValue(), 0.5 * d_visuScale.getValue());

        vparams->drawTool()->drawArrow(P - dir * log(f.norm()+1)*d_visuScale.getValue(), P, 
                                        log(f.norm()+1)*d_visuScale.getValue()/20.0, 
                                        sofa::type::RGBAColor::red());
    }
}

} // namespace