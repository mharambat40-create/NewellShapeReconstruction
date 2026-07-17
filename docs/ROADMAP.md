# Newell Roadmap

## Current state

The repository now has:

- a root-level CMake project;
- a buildable Qt 6 Widgets application shell;
- a `src/` layout aligned with an MVC-inspired structure;
- placeholder directories for model, controller and infrastructure modules;
- project documentation under `docs/`.

What is still missing:

- smoothing and fitting algorithms;
- evaluation metrics;
- visual comparison tools beyond the initial viewport;
- OCCT or other geometry-processing dependencies.
- richer geometry formats beyond ASCII PLY.

## Phase 1: Foundation slice

Goals:

- Stabilise the new repository layout
- Keep the Qt shell buildable
- Add the first non-UI domain types

Deliverables:

- Root `CMakeLists.txt` using C++20
- Basic `src/model/geometry` types
- Initial controller skeleton

Status:

- Completed for the first import-oriented slice.
- Extended with a basic point-cloud viewport and camera controls.

## Phase 2: Import and data preservation

Goals:

- Import one practical geometry format first
- Preserve original geometry and provenance

Recommended first format:

- `.ply`

Deliverables:

- `src/model/io` import service
- Internal point cloud representation
- Explicit units and tolerance handling

Status:

- Initial ASCII `.ply` point-cloud import is implemented.
- Original and current geometry are preserved through `GeometryDocument`.
- Imported point clouds can now be rendered in a Qt OpenGL viewport for inspection.

## Phase 3: Initial visualisation

Goals:

- Replace the empty main window shell with meaningful geometry presentation
- Add viewport and object inspection foundations

Deliverables:

- Viewport widget or rendering surface
- Scene-to-view adapter
- Original-versus-current visibility toggles

## Phase 4: Pre-processing and smoothing baseline

Goals:

- Add non-destructive processing
- Establish parameter tracking and result history

Deliverables:

- Pre-processing hooks
- Initial Laplacian smoothing

Current reconstruction foundation delivered alongside preprocessing:

- Ball Pivoting and Greedy Projection point-cloud reconstruction;
- Voxel field construction and standard Marching Cubes extraction;
- bounded oriented-normal RBF backend;
- optional CGAL-backed Delaunay and Alpha Shapes with PCA planarity checks and 2D/2.5D output;
- central registry, method requirements, cancellation, progress and mesh diagnostics.

Normal-estimation foundation now delivered and exposed:

- fixed-radius, multi-scale and quadratic local normal estimation;
- method-specific configuration, availability rules, confidence and diagnostics;
- direct use by Ball Pivoting and Greedy Projection without claiming global orientation.

Orientation and field-derived interfaces are reserved internally for later
reintroduction. They are not shown in the Normal Selection dialog because the
current document/controller does not persist existing normal fields, implicit
fields or voxel scalar fields. Oriented-normal RBF and Screened Poisson remain
unavailable in this workflow.

Screened Poisson remains dependency-blocked, direct Marching Cubes requires scalar-field input, and NURBS/B-Spline work remains in Parametric Fitting.
- Background execution for longer operations

## Phase 5: Evaluation and comparison

Goals:

- Quantify changes after smoothing
- Support research-oriented comparison workflows

Deliverables:

- RMS deviation
- Maximum deviation
- Visual comparison overlays or side-by-side inspection

## Phase 6: Parametric fitting

Goals:

- Fit simple surfaces to selected regions
- Prepare downstream CAD reconstruction

Deliverables:

- Plane fitting
- Cylinder fitting
- Sphere fitting

## Phase 7: CAD reconstruction and export

Goals:

- Introduce CAD-kernel-backed reconstruction only when the earlier layers justify it
- Add CAD-compatible exchange

Deliverables:

- OCCT integration under `src/infrastructure/occt`
- CAD reconstruction bridge
- STEP and IGES export path

## Dependency timing

- Start with Qt 6, CMake, C++20 and Eigen.
- Add mesh or point-cloud libraries only when the current internal model shows a real limitation.
- Add OCCT later for CAD reconstruction and exchange, not as the first smoothing dependency.

## Recommended next implementation step

Persist normal fields and scalar-field context in `GeometryDocument`, then add
an explicit optional orientation stage separate from estimation. Only after
that state exists should viewpoint, graph and field-derived methods return to
the UI.
