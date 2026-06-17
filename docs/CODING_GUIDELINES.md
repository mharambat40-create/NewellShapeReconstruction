# Newell Coding Guidelines

## Scope

These guidelines apply to the Newell prototype as it grows from the current Qt application shell into a geometry-processing research tool.

## Architectural rules

- Keep source code under `src/`.
- Keep documentation under `docs/`.
- Keep generated build output out of source directories.
- Keep geometry-processing code independent from Qt widget classes.
- Avoid circular dependencies across `model`, `view` and `controller`.
- Preserve future extensibility for smoothing, fitting, OCCT and export workflows.

## MVC rules

### Model

- Contains geometry, smoothing, fitting, evaluation and import/export logic.
- Must not depend on Qt widgets.
- Should expose UI-independent data types and service interfaces.

### View

- Contains Qt UI classes and `.ui` files.
- Must not contain geometry-processing algorithms.
- Should focus on presentation, interaction and rendering adapters.

### Controller

- Connects user actions to model services.
- Owns application state and workflow coordination.
- Should not duplicate domain logic from the model.

## C++20 rules

- Use RAII for ownership and cleanup.
- Prefer smart pointers to raw owning pointers.
- Apply `const` correctness consistently.
- Prefer `std::vector` for dynamic contiguous storage.
- Use `std::optional` for values that may be absent.
- Use `std::filesystem` for path handling.
- Prefer standard library facilities before adding custom infrastructure.
- Keep classes small and focused.

## Geometry-processing rules

- Distinguish point clouds, meshes and CAD-oriented geometry explicitly.
- Preserve original imported geometry for comparison.
- Avoid destructive overwrite unless explicitly requested.
- Keep units, tolerances and algorithm parameters explicit.
- Record provenance for derived geometry where practical.
- Assume scanned data may be noisy, incomplete or topologically inconsistent.

## UI and threading rules

- Do not run long computations on the UI thread.
- Route progress, cancellation and error reporting through controller-facing application services.
- Keep signals and slots concentrated at View/Controller boundaries.

## Build and repository hygiene

- Keep `README.md` at the repository root.
- Do not treat `build/` as source code.
- Keep `CMakeLists.txt` aligned with the real source layout.
- Update docs whenever the structure or dependency strategy changes materially.

## Testing expectations

- Add unit tests for geometry and numerical code.
- Add integration tests for import/export and end-to-end slices.
- Compare numerical results with explicit tolerances, not exact floating-point equality.
- Use simple reference geometry and sample datasets to build regression coverage over time.
