# Newell

Newell is a Qt/C++20 desktop research prototype for shape improvement using surface smoothing. The project is being organised as a modular engineering application that will import scanned or CAD-like geometry, visualise it, apply smoothing workflows, compare results against the original geometry, fit parametric surfaces to selected regions, and later export CAD-compatible results.

## Current state

The repository currently contains:

- a buildable Qt 6 Widgets application shell;
- a cleaned `src/` layout with `app`, `view`, `model`, `controller` and `infrastructure` roots;
- project documentation under `docs/`;
- placeholder directories for future geometry, smoothing, fitting, evaluation, CAD and export work.

The current implementation is intentionally minimal. Geometry processing, CAD reconstruction and export are not implemented yet.

## Repository layout

```text
.
├── CMakeLists.txt
├── README.md
├── docs/
├── src/
│   ├── app/
│   ├── controller/
│   ├── infrastructure/
│   ├── model/
│   └── view/
├── tests/
└── assets/
```

## Build

Configure and build with CMake:

```bash
cmake -S . -B build
cmake --build build
```

Qt 6 Core and Widgets are currently required. See [docs/DEPENDENCIES.md](/Users/martin/Desktop/Newell/docs/DEPENDENCIES.md:1) for the planned dependency strategy.

## Architecture direction

The project follows an MVC-inspired separation:

- Model: geometry data, smoothing, fitting, evaluation and import/export logic
- View: Qt UI, main window, viewport and panels
- Controller: commands, application state and coordination between UI and model services

See [docs/PROJECT_ARCHITECTURE.md](/Users/martin/Desktop/Newell/docs/PROJECT_ARCHITECTURE.md:1) for details.
