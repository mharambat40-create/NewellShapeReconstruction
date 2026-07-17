#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_MESHVALIDATION_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_MESHVALIDATION_H

#include "model/geometry/TriangleMesh.h"

#include <cstddef>
#include <string>

struct MeshDiagnostics
{
    std::size_t vertexCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t removedDegenerateTriangleCount = 0U;
    std::size_t removedDuplicateTriangleCount = 0U;
    std::size_t boundaryEdgeCount = 0U;
    std::size_t nonManifoldEdgeCount = 0U;
    std::size_t connectedComponentCount = 0U;
};

struct MeshValidationResult
{
    bool valid = false;
    TriangleMesh mesh;
    MeshDiagnostics diagnostics;
    std::string errorMessage;
};

[[nodiscard]] MeshValidationResult validateMesh(
    TriangleMesh mesh,
    double geometricTolerance = 1.0e-9);

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_MESHVALIDATION_H
