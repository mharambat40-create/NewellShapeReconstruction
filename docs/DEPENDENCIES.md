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
- optional CGAL feature detection through `NEWELL_ENABLE_CGAL`

Not yet integrated:

- OpenMesh
- libigl
- a screened octree Poisson backend
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
- adds model/import, preprocessing, reconstruction and normal-processing test executables.

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
- symmetric covariance eigendecomposition for local normal estimation
- SVD least-squares fitting for quadratic local surfaces
- Least-squares fitting
- Numerical support for geometry processing

Current status:

- Required by the first import-oriented slice
- Linked through `Eigen3::Eigen`
- Used by the four point-cloud estimators exposed in the Normal Selection workflow

The normal-processing slice adds no third-party dependency. Spatial queries
reuse Newell's existing KD-tree. Reserved orientation and field-derived
extension points also rely only on Eigen and standard-library containers.

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

Current status:

- optional and detected in the current macOS/Homebrew development environment;
- detected with `find_package(CGAL CONFIG QUIET)` when `NEWELL_ENABLE_CGAL=ON`;
- enables Delaunay and Alpha Shapes reconstruction in 2D and 2.5D modes;
- unavailable methods remain disabled with an installation/configuration reason when CGAL is absent.

On macOS, install the optional backend with:

```bash
brew install cgal
```

### Screened Poisson

`NEWELL_ENABLE_POISSON` reserves the build boundary for a future screened octree implementation. It is off by default, and enabling it currently produces a clear configure-time error because no credible provider is configured. The project does not substitute RBF, voxel reconstruction or ordinary Poisson reconstruction for Screened Poisson.

### Reconstruction resource limits

- `ScalarGrid3D` validates dimensions, multiplication overflow and a configurable maximum voxel count before allocation.
- Voxel reconstruction defaults to a 16-million-voxel ceiling.
- RBF reconstruction limits control points to 512, defaults to 128, limits sampled voxels and caps total field evaluations.
- Delaunay/Alpha memory is owned inside the optional CGAL adapter; Greedy Projection reuses the shared KD-tree and bounds each neighbourhood.

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
