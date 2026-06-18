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

Smoothing, fitting and CAD reconstruction are still future work.

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
