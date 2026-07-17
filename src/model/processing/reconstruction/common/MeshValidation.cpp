#include "model/processing/reconstruction/common/MeshValidation.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

namespace
{
using Edge = std::array<std::size_t, 2>;
using Face = std::array<std::size_t, 3>;

Edge canonicalEdge(std::size_t first, std::size_t second)
{
    return {std::min(first, second), std::max(first, second)};
}

Face canonicalFace(const Triangle &triangle)
{
    Face face = triangle.vertexIndices;
    std::sort(face.begin(), face.end());
    return face;
}

class DisjointSet
{
public:
    explicit DisjointSet(std::size_t size)
        : parent_(size)
    {
        std::iota(parent_.begin(), parent_.end(), 0U);
    }

    std::size_t find(std::size_t index)
    {
        if (parent_[index] != index) {
            parent_[index] = find(parent_[index]);
        }
        return parent_[index];
    }

    void unite(std::size_t first, std::size_t second)
    {
        first = find(first);
        second = find(second);
        if (first != second) {
            parent_[second] = first;
        }
    }

private:
    std::vector<std::size_t> parent_;
};
}

MeshValidationResult validateMesh(TriangleMesh mesh, double geometricTolerance)
{
    MeshValidationResult result;
    if (!std::isfinite(geometricTolerance) || geometricTolerance <= 0.0) {
        result.errorMessage = "Mesh-validation tolerance must be positive and finite.";
        return result;
    }

    for (const Point3d &vertex : mesh.vertices()) {
        if (!vertex.vector().allFinite()) {
            result.errorMessage = "Mesh contains a non-finite vertex.";
            return result;
        }
    }

    TriangleMesh cleaned;
    cleaned.setVertices(mesh.vertices());
    std::set<Face> uniqueFaces;
    const double minimumAreaTwice = geometricTolerance * geometricTolerance;
    for (const Triangle &triangle : mesh.triangles()) {
        const auto &indices = triangle.vertexIndices;
        if (indices[0] >= mesh.vertexCount() || indices[1] >= mesh.vertexCount() ||
            indices[2] >= mesh.vertexCount()) {
            result.errorMessage = "Mesh contains an out-of-bounds triangle index.";
            return result;
        }
        if (indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0]) {
            ++result.diagnostics.removedDegenerateTriangleCount;
            continue;
        }
        const Eigen::Vector3d &first = mesh.vertices()[indices[0]].vector();
        const Eigen::Vector3d &second = mesh.vertices()[indices[1]].vector();
        const Eigen::Vector3d &third = mesh.vertices()[indices[2]].vector();
        if ((second - first).cross(third - first).norm() <= minimumAreaTwice) {
            ++result.diagnostics.removedDegenerateTriangleCount;
            continue;
        }
        if (!uniqueFaces.insert(canonicalFace(triangle)).second) {
            ++result.diagnostics.removedDuplicateTriangleCount;
            continue;
        }
        cleaned.addTriangle(indices[0], indices[1], indices[2]);
    }

    if (cleaned.empty()) {
        result.errorMessage = "Mesh contains no valid triangles.";
        return result;
    }

    std::map<Edge, std::vector<std::size_t>> edgeFaces;
    DisjointSet components(cleaned.triangleCount());
    for (std::size_t faceIndex = 0; faceIndex < cleaned.triangleCount(); ++faceIndex) {
        const auto &indices = cleaned.triangles()[faceIndex].vertexIndices;
        for (const Edge edge : {
                 canonicalEdge(indices[0], indices[1]),
                 canonicalEdge(indices[1], indices[2]),
                 canonicalEdge(indices[2], indices[0])}) {
            std::vector<std::size_t> &incidentFaces = edgeFaces[edge];
            if (!incidentFaces.empty()) {
                components.unite(faceIndex, incidentFaces.front());
            }
            incidentFaces.push_back(faceIndex);
        }
    }

    for (const auto &[edge, incidentFaces] : edgeFaces) {
        (void)edge;
        if (incidentFaces.size() == 1U) {
            ++result.diagnostics.boundaryEdgeCount;
        } else if (incidentFaces.size() > 2U) {
            ++result.diagnostics.nonManifoldEdgeCount;
        }
    }

    std::set<std::size_t> roots;
    for (std::size_t faceIndex = 0; faceIndex < cleaned.triangleCount(); ++faceIndex) {
        roots.insert(components.find(faceIndex));
    }

    result.mesh = std::move(cleaned);
    result.diagnostics.vertexCount = result.mesh.vertexCount();
    result.diagnostics.triangleCount = result.mesh.triangleCount();
    result.diagnostics.connectedComponentCount = roots.size();
    result.valid = result.mesh.hasValidIndices();
    if (!result.valid) {
        result.errorMessage = "Mesh indices failed final validation.";
    }
    return result;
}
