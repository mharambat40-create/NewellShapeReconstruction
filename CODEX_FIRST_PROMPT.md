# Codex Prompt — Newell Project Architecture Setup

You are working on a Qt/C++20 desktop application named **Newell**.

Clone and use the repository locally:

```bash
cd ~/Desktop
git clone https://github.com/mharambat40-create/Newell.git
cd Newell
```

Inspect the repository before modifying anything. Report the current structure, build system, Qt configuration, and whether OCCT or other geometry dependencies are already present.

## Objective

Newell is a research-oriented engineering prototype for **shape improvement using surface smoothing**. It shall import scanned or CAD-like geometry, visualise it, apply smoothing algorithms, evaluate the result, associate selected regions with parametric surfaces, and export processed geometry to CAD-compatible formats.

Workflow:

```text
Import geometry -> Pre-processing -> Visualisation -> Smoothing and shape improvement -> Quality evaluation -> Parametric surface fitting -> CAD reconstruction -> STEP / IGES / mesh export
```

## Task

Create these files:

```text
docs/PROJECT_ARCHITECTURE.md
docs/CODING_GUIDELINES.md
docs/ROADMAP.md
docs/DEPENDENCIES.md
```

Create `docs/` if needed. Do not implement the full application yet. This first task is architectural documentation and preparation only.

## Architecture

Use an MVC-inspired architecture:

- **Model**: point clouds, meshes, CAD wrappers, smoothing algorithms, fitting algorithms, evaluation metrics, import/export services.
- **View**: Qt UI, main window, 3D viewport, panels, visual comparison tools.
- **Controller**: user actions, commands, application state, coordination between UI and model services.

The Model must not depend on Qt widgets. Geometry algorithms must remain independent from the UI.

## Modules to document

- Import: `.ply`, optional `.stl`/`.obj`, later `.iges`/`.step`.
- Geometry core: point clouds, meshes, surfaces, CAD entities.
- Visualisation: point, mesh, surface, wireframe, shaded, normals, deviation maps.
- Pre-processing: cleaning, outlier removal, normal estimation, optional meshing.
- Smoothing: Laplacian first, then Taubin/constrained smoothing, later energy-based fairing.
- Evaluation: RMS deviation, max deviation, Hausdorff approximation, curvature variation, computation time.
- Parametric fitting: plane, cylinder, sphere, later B-spline/NURBS.
- CAD reconstruction: OCCT-based conversion to CAD entities.
- Export: `.ply`, `.stl`, `.obj`, `.iges`, `.step`.

## Technology strategy

Use Qt 6 for UI, C++20 as language, CMake as build system, Eigen for numerical computation, and OCCT for CAD surfaces, topology, STEP/IGES import/export. Consider OpenMesh, libigl, CGAL or PCL only when needed. Do not use OCCT as the primary point cloud smoothing library.

## Suggested structure

```text
Newell/
├── CMakeLists.txt
├── README.md
├── docs/
├── src/
│   ├── app/
│   ├── model/
│   │   ├── geometry/
│   │   ├── smoothing/
│   │   ├── fitting/
│   │   ├── evaluation/
│   │   └── io/
│   ├── view/
│   ├── controller/
│   ├── infrastructure/
│   │   ├── occt/
│   │   ├── mesh/
│   │   └── logging/
│   └── main.cpp
├── tests/
├── assets/
└── third_party/
```

Adapt this structure to the existing repository if needed, while preserving separation of responsibilities.

## Rules

- Keep geometry processing independent from Qt widgets.
- Preserve original geometry for comparison.
- Avoid destructive operations unless explicitly requested.
- Store smoothing parameters with generated results.
- Keep tolerances explicit and configurable.
- Avoid long computations on the UI thread.
- Use RAII, smart pointers, const-correctness, `std::vector`, `std::optional`, and `std::filesystem`.
- Prefer composition over inheritance.
- Do not commit changes unless explicitly instructed.

## Final Codex report

Report: repository structure, build system, Qt status, OCCT status, files created/modified, assumptions, risks or missing dependencies, and the next recommended implementation step.
