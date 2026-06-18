# Newell Dependencies

## Current dependency status

Currently integrated:

- Qt 6 Core
- Qt 6 Widgets
- Qt 6 OpenGL
- Qt 6 OpenGLWidgets
- CMake
- C++20 through the root build configuration
- Eigen through the `NewellCore` target configuration

Not yet integrated:

- OpenMesh
- libigl
- CGAL
- PCL
- OCCT

## Current build configuration

The active build entry point is the root [CMakeLists.txt](/Users/martin/Desktop/Newell/CMakeLists.txt:1).

It currently:

- defines the `Newell` project;
- enforces C++20;
- locates Qt 6.5 Core and Widgets;
- prefers `find_package(Eigen3)` and falls back to common Homebrew include paths;
- builds the application from `src/app` and `src/view`;
- builds a Qt-free `NewellCore` library for model and controller code;
- links Qt OpenGL modules for the point-cloud viewport;
- adds a small `NewellTests` executable.

## Primary technologies

### Qt 6

Role:

- GUI framework
- Application lifecycle
- Signals and slots
- Main window and future viewport integration

Current status:

- Integrated
- Used for `QOpenGLWidget`-based point-cloud rendering in the View layer

### C++20

Role:

- Main implementation language
- Standard library features for ownership, containers, optional values and filesystem

Current status:

- Integrated

### CMake

Role:

- Build system
- Source layout definition
- Future test and dependency integration

Current status:

- Integrated

### Eigen

Role:

- Linear algebra
- PCA
- Least-squares fitting
- Numerical support for geometry processing

Current status:

- Required by the first import-oriented slice
- Linked through `Eigen3::Eigen`

## Candidate future geometry libraries

### OpenMesh

Useful for:

- Mesh topology
- Half-edge structures
- Mesh-based smoothing workflows

### libigl

Useful for:

- Geometry-processing prototyping
- Lightweight algorithm access on top of Eigen

### CGAL

Useful for:

- Robust computational geometry
- Exact predicates or specialised geometric algorithms

### PCL

Useful for:

- Point cloud filtering
- Normal estimation
- Segmentation and scanner-oriented workflows

These libraries should be added only when a concrete model or algorithm requirement justifies them.

## OCCT strategy

### OCCT

Planned role:

- STEP and IGES import/export
- B-spline and NURBS surface support
- CAD topology such as faces, edges, wires and shells
- Parametric CAD reconstruction from fitted results

Current status:

- Not integrated
- Reserved structurally under `src/infrastructure/occt`

Important rule:

- Do not use OCCT as the primary point cloud smoothing library.

## Recommended dependency order

1. Keep the current Qt 6 and C++20 shell stable.
2. Add Eigen.
3. Build internal geometry and import support.
4. Add OpenMesh, libigl, CGAL or PCL only if real algorithm work demands them.
5. Add OCCT later for CAD reconstruction and exchange.
