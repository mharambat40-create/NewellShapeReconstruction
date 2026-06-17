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
- viewport and visual comparison tools;
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

Add the next vertical slice:

1. Introduce a basic viewport placeholder or scene summary panel.
2. Surface imported geometry metadata in the UI beyond the status bar.
3. Add pre-processing hooks and document-level result history without implementing smoothing yet.

That step builds on the import slice without jumping prematurely into OCCT or full visualisation.
