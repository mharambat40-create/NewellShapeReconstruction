# Newell Project Architecture

## Purpose

Newell is a Qt/C++20 desktop prototype for research on shape improvement using surface smoothing. The system is intended to import scanned or CAD-like geometry, visualise it, apply smoothing or fitting operations, evaluate the result, reconstruct CAD-oriented surfaces where appropriate, and export compatible outputs later.

The project is a research prototype, not a full production CAD system.

## Current implementation baseline

The repository now contains a buildable Qt 6 Widgets shell organised at the repository root:

```text
.
├── CMakeLists.txt
├── README.md
├── docs/
├── src/
│   ├── app/
│   │   └── main.cpp
│   ├── controller/
│   ├── infrastructure/
│   │   ├── logging/
│   │   ├── mesh/
│   │   └── occt/
│   ├── model/
│   │   ├── evaluation/
│   │   ├── fitting/
│   │   ├── geometry/
│   │   ├── io/
│   │   └── smoothing/
│   └── view/
│       ├── MainWindow.cpp
│       ├── MainWindow.h
│       └── MainWindow.ui
├── tests/
└── assets/
    └── sample_geometry/
```

The first vertical slice is now implemented:

- ASCII PLY import in `src/model/io`
- `Point3d`, `PointCloud` and `GeometryDocument` in `src/model/geometry`
- controller-owned document state in `src/controller`
- a Qt import action in `src/view`

The second vertical slice now extends that flow with basic point-cloud visualisation:

- `BoundingBox3d` in `src/model/geometry`
- `PointCloudViewport` in `src/view`
- basic mouse rotation, zoom and pan for imported point clouds

Smoothing, parametric fitting and CAD reconstruction are still future work. A
Qt-independent mesh-reconstruction and normal-processing foundation is now
available for the Convert to surface workflow.

## Processing workflow

```text
Import -> Geometry Core -> Visualisation -> Pre-processing -> Smoothing ->
Evaluation -> Parametric Fitting -> CAD Reconstruction -> Export
```

This is a workflow model, not a hard pipeline. The application must preserve the original geometry and support repeated experiments, side-by-side comparison and parameter variation.

## MVC-inspired separation

### Model

The Model layer contains geometry data, algorithms and non-UI services.

Responsibilities:

- Point cloud, mesh and CAD-oriented data structures
- Import and export services
- Pre-processing operations
- Smoothing algorithms
- Parametric fitting algorithms
- Evaluation metrics and analysis results

Rules:

- The Model must not depend on Qt widget classes.
- Geometry processing must stay testable without launching the UI.
- Results should preserve provenance, parameters and tolerances.

### View

The View layer contains Qt-facing user interface elements.

Current content:

- `src/view/MainWindow.cpp`
- `src/view/MainWindow.h`
- `src/view/MainWindow.ui`
- `src/view/FloatingWorkflowMenu.cpp`
- `src/view/FloatingWorkflowMenu.h`
- `src/view/PointCloudViewport.cpp`
- `src/view/PointCloudViewport.h`

Future responsibilities:

- Main window
- 3D viewport
- Panels and inspectors
- Visual comparison tools
- Parameter editors and progress display

Rules:

- The View must not contain smoothing, fitting or evaluation logic.
- Rendering and camera interaction may live in the View, but geometry analysis utilities must remain in the Model.

### Controller

The Controller layer connects user actions to application logic and state.

Current responsibilities:

- load an ASCII PLY file into a `GeometryDocument`
- preserve original and current geometry states
- expose basic loaded status and point count

Future responsibilities:

- Import commands
- Smoothing commands
- Fitting commands
- Export commands
- Selection and region-of-interest state
- Coordination between View and Model services
- Undo/redo and background task orchestration later

## Processing tasks and progress

Existing preprocessing selection and surface reconstruction follow this flow:

```text
Qt View -> Controller -> Pipeline -> Model algorithm -> Result
```

The Qt-independent processing contract lives in `src/model/processing/common`. Algorithms receive a progress callback and cancellation token; pipelines map local algorithm progress into global phases and reserve 100% for a validated result. `MainWindow` runs non-trivial work with `QtConcurrent` and queues progress updates back to the UI thread.

`ProcessingProgressWidget` provides the shared determinate bar and fixed-width percentage label used by Remove invalid points, Downsampling and Convert to surface. `ProgressReporter` clamps non-finite and out-of-range values, prevents regressions, and throttles callbacks to a changed integer percentage no more frequently than every 50 ms (plus terminal state changes).

### Surface reconstruction backends

`SurfaceReconstructorRegistry` is the single factory and availability source for surface reconstruction. Every backend implements `ISurfaceReconstructor`, declares its input and normal requirements, receives a method-specific parameter variant, and uses the shared progress/cancellation contracts.

The current backend families are:

- direct/local: CGAL-backed Delaunay and Alpha Shapes with shared PCA projection, 2D/2.5D reconstruction and explicit planarity rejection, plus Ball Pivoting and Greedy Projection;
- volumetric: checked `ScalarGrid3D`, Voxel reconstruction and standard 256-case Marching Cubes;
- implicit: bounded oriented-normal RBF reconstruction followed by Marching Cubes;
- external adapter: Screened Poisson remains unavailable until a screened octree solver is configured.

All generated meshes pass through shared validation for finite vertices, valid indices, degenerate and duplicate faces, boundary/non-manifold edges and connected components. NURBS/B-Spline fitting is intentionally excluded from mesh reconstruction and remains part of the later Parametric Fitting stage.

The Convert to surface view uses `MethodSelectionDialog` for reconstruction and
normal-method choices. The submenu initially displays `None` and keeps its
dynamic parameter area empty; after confirmation, it inserts only controls owned
by the selected backend. Convert is enabled only when a valid point cloud,
backend, normal configuration and method-specific planarity requirement are
available.

### Normal processing

Normal processing lives in `src/model/processing/normals` and is independent of
Qt widgets. The current point-cloud workflow exposes exactly four estimators
through `NormalMethodRegistry`:

- estimation from points: PCA k-nearest, PCA fixed radius, multi-scale PCA and quadratic local fitting;

The `INormalOrienter` and `IFieldNormalGenerator` extension points remain in the
Model for later workflows, but their methods are not mapped into the current
Normal Selection dialog. Newell does not yet persist an existing normal field,
implicit field or scalar grid that would make those choices actionable.

`NormalField` stores a normal, validity and bounded confidence per point, plus
an explicit `consistentlyOriented` flag. PCA and quadratic estimators leave that
flag false. Ball Pivoting and Greedy Projection consume these estimated normals
without claiming global orientation. RBF and Screened Poisson require
consistently oriented normals and therefore remain unavailable in the current
point-cloud UI workflow.

The shared local-surface utility owns centroid, covariance, symmetric
eigendecomposition, degeneracy and confidence logic. Method-specific parameters
use a `std::variant`, and all methods use shared progress, cancellation and
diagnostics contracts. Worker progress is limited to 99%; the UI publishes 100%
only after accepting the complete result.

Reserved graph propagation uses a sparse k-nearest-neighbour minimum spanning
forest, while reserved field-gradient implementations support continuous and
sampled scalar fields. These components are intentionally not user-facing until
the document and controller can supply their required input state.

## Module layout

### `src/app`

Application entry point and high-level startup wiring.

Current file:

- `main.cpp`

Current interaction:

- launches the Qt shell that can trigger an ASCII PLY import
- configures the OpenGL surface format for the point-cloud viewport

### `src/model/geometry`

Current content:

- `Point3d`
- `PointCloud`
- `GeometryDocument`

Current responsibility:

- preserve both original and current point-cloud state so future smoothing can modify the current geometry without destroying the imported reference
- compute non-UI geometry helpers such as point-cloud bounding boxes for view fitting

Future additions:

- Meshes
- CAD wrappers
- Bounding boxes, units and tolerances
- Common geometry utilities

### `src/model/io`

Current content:

- ASCII `.ply` point-cloud importer

Current responsibility:

- read ASCII PLY headers
- parse vertex count
- read x, y, z coordinates
- reject binary PLY files and malformed input with clear errors

Future additions:

- Geometry import
- Geometry export
- File format adapters
- Provenance metadata capture

### `src/model/smoothing`

Planned home for:

- Laplacian smoothing
- Taubin smoothing
- Feature-aware or constrained smoothing later

### `src/model/fitting`

Planned home for:

- Planes
- Cylinders
- Spheres
- Later B-spline or NURBS fitting support

### `src/model/evaluation`

Planned home for:

- RMS deviation
- Maximum deviation
- Approximate Hausdorff distance
- Curvature-based quality indicators

### `src/infrastructure/mesh`

Reserved for lower-level mesh-specific helpers or third-party integration seams if a dedicated mesh library is introduced later.

### `src/infrastructure/occt`

Reserved for future OCCT integration:

- STEP/IGES exchange
- CAD topology
- B-spline/NURBS surface construction
- CAD reconstruction bridges

### `src/infrastructure/logging`

Reserved for application logging, diagnostics and trace output.

## Architectural rules

- Keep geometry processing independent from Qt widgets.
- Preserve original geometry for comparison.
- Avoid destructive operations unless explicitly requested.
- Keep tolerances and units explicit.
- Avoid circular dependencies between `model`, `view` and `controller`.
- Run long computations off the UI thread.
- Keep generated build output outside the source tree in the long term.

## Technology strategy

- Qt 6 for GUI, application framework, signals and slots
- C++20 for implementation
- CMake for build configuration
- Eigen as the first numerical dependency
- OpenMesh, libigl, CGAL or PCL only when a concrete need appears
- OCCT later for CAD surfaces, topology, STEP/IGES exchange, B-spline/NURBS support and CAD reconstruction

OCCT must not become the primary point cloud smoothing library. Smoothing and most numerical processing should remain independent from the CAD kernel.
