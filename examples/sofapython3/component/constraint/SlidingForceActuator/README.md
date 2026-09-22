# Sliding force actuators

Examples for the four actuators that estimate where a force is applied on a surface, and how large it is.
They all solve the same problem and differ only in how the location is parameterized:

| Scene | Component | Location |
|---|---|---|
| `SlidingForceActuator.py` | `SlidingForceActuator` | local coordinates in the current triangle |
| `SmoothSlidingForceActuator.py` | `SmoothSlidingForceActuator` | same as above, with low-pass filtering of the sliding direction and step (`dirMomentum`, `slideMomentum`) |
| `SphericalSlidingForceActuator.py` | `SphericalSlidingForceActuator` | global (theta, phi) on a spherical parameterization of the surface |
| `AreaContactSlidingForceActuator.py` | `AreaContactSlidingForceActuator` | (theta, phi) plus a contact radius; estimates a pressure patch instead of a point force |

`SlidingForceActuator` parameterizes the location as Cartesian coordinates (U, V) in the tangent plane of its current triangle. `SmoothSlidingForceActuator` uses barycentric weights (wB, wC) instead.
In one triangle, the two parameterizations are equivalent.

## Running

    runSofa SlidingForceActuator.py              # GUI

Upon running the scene, two soft spheres are shown side by side:

- Right, `groundTruth`: forward simulation. A known force (red point) pushes on the sphere and 16 markers (green)
  record the deformation.
- Left, `estimation`: inverse simulation. A `PositionEffector` drives the same markers (yellow) toward the
  positions measured on `groundTruth`, and the actuator (force arrow) starts about 22 mm away from the true location
  and has to slide to it.

A controller in `common.py` copies the marker positions of `groundTruth` into the effector goals at every step and
prints the location error and the estimated force every 20 steps.

<!-- ## Making your own scene

`common.py` holds the shared setup: `addHeader` (plugins, `FreeMotionAnimationLoop`, `QPInverseProblemSolver`),
`addTruth`, `addEstimate` and the controller. `addEstimate` returns the node in which the actuator has to be
added. That node holds the surface mesh (`MeshTopology` + `MechanicalObject`) the actuator slides on and is
mapped to the FEM model with a `BarycentricMapping`, added after the actuator.

What each actuator needs:

- `SlidingForceActuator`, `SmoothSlidingForceActuator`: the triangle to start on (`triangleIndices`).
  `closestTriangle()` finds it from a point.
- `SphericalSlidingForceActuator`, `AreaContactSlidingForceActuator`: a `.spar` file of the surface (`sparFile`)
  and the starting angles (`initTheta`, `initPhi`). `sparAngles()` converts a direction in mesh coordinates to
  these angles. The area contact actuator also takes a starting radius (`initRadius`).

Force Data of the actuators (`initForce`, `minForce`, `maxForce`, `currentForces`) hold the constraint multiplier
of the QP, which is force * dt with `EulerImplicitSolver`. `AreaContactSlidingForceActuator` works with a pressure,
so its fields hold pressure * dt. The controller converts before printing. Units of the scenes are mm, kg, s. -->

## Meshes

We used a sphere of radius 30 mm, it was generated using scripts:

    python generate_sphere_mesh.py                                        # gmsh, meshio
  
For the spherical parameterization, one additional step is needed to create the parameterization map:

    python build_spherical_param.py --mesh mesh/sphere.stl --pole 0,0,-1   # numpy, trimesh

`mesh/sphere.vtk` is the FEM model (6 mm tetrahedra), `mesh/sphere.stl` the surface the actuators slide on
(2.5 mm triangles), `mesh/sphere.spar` its spherical parameterization. The `.spar` file stores the triangles of
the STL, so rebuild it whenever the STL changes (about 4 minutes for this mesh). `build_spherical_param.py`
works on any closed genus-0 surface. `--pole` is the mesh direction mapped to theta = 0; point it at a region
the actuator does not visit, since the parameterization is singular at its poles (here the fixed base).
