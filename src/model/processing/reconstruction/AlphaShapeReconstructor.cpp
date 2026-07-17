#include "model/processing/reconstruction/AlphaShapeReconstructor.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/reconstruction/common/ProjectionPlaneUtilities.h"

#include <Eigen/Geometry>

#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#if NEWELL_HAS_CGAL
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#endif

namespace
{
#if NEWELL_HAS_CGAL
using MeshEdge = std::pair<std::size_t, std::size_t>;

MeshEdge canonicalEdge(std::size_t first, std::size_t second)
{
    return std::minmax(first, second);
}

TriangleMesh filterMeshComponents(
    const TriangleMesh &mesh,
    bool keepLargestComponentOnly,
    double minimumComponentArea)
{
    if (!keepLargestComponentOnly && minimumComponentArea <= 0.0) {
        return mesh;
    }

    std::map<MeshEdge, std::vector<std::size_t>> edgeFaces;
    for (std::size_t faceIndex = 0; faceIndex < mesh.triangleCount(); ++faceIndex) {
        const auto &indices = mesh.triangles()[faceIndex].vertexIndices;
        edgeFaces[canonicalEdge(indices[0], indices[1])].push_back(faceIndex);
        edgeFaces[canonicalEdge(indices[1], indices[2])].push_back(faceIndex);
        edgeFaces[canonicalEdge(indices[2], indices[0])].push_back(faceIndex);
    }

    std::vector<std::vector<std::size_t>> adjacency(mesh.triangleCount());
    for (const auto &[edge, faces] : edgeFaces) {
        (void)edge;
        for (std::size_t first = 0; first < faces.size(); ++first) {
            for (std::size_t second = first + 1U; second < faces.size(); ++second) {
                adjacency[faces[first]].push_back(faces[second]);
                adjacency[faces[second]].push_back(faces[first]);
            }
        }
    }

    std::vector<std::size_t> componentByFace(mesh.triangleCount(), mesh.triangleCount());
    std::vector<double> componentAreas;
    for (std::size_t seed = 0; seed < mesh.triangleCount(); ++seed) {
        if (componentByFace[seed] != mesh.triangleCount()) {
            continue;
        }
        const std::size_t component = componentAreas.size();
        componentAreas.push_back(0.0);
        std::queue<std::size_t> pending;
        pending.push(seed);
        componentByFace[seed] = component;
        while (!pending.empty()) {
            const std::size_t faceIndex = pending.front();
            pending.pop();
            const auto &indices = mesh.triangles()[faceIndex].vertexIndices;
            const Eigen::Vector3d &first = mesh.vertices()[indices[0]].vector();
            const Eigen::Vector3d &second = mesh.vertices()[indices[1]].vector();
            const Eigen::Vector3d &third = mesh.vertices()[indices[2]].vector();
            componentAreas[component] += 0.5 * (second - first).cross(third - first).norm();
            for (const std::size_t neighbour : adjacency[faceIndex]) {
                if (componentByFace[neighbour] == mesh.triangleCount()) {
                    componentByFace[neighbour] = component;
                    pending.push(neighbour);
                }
            }
        }
    }

    const std::size_t largestComponent = static_cast<std::size_t>(std::distance(
        componentAreas.begin(),
        std::max_element(componentAreas.begin(), componentAreas.end())));
    TriangleMesh filtered;
    filtered.setVertices(mesh.vertices());
    for (std::size_t faceIndex = 0; faceIndex < mesh.triangleCount(); ++faceIndex) {
        const std::size_t component = componentByFace[faceIndex];
        if ((keepLargestComponentOnly && component != largestComponent) ||
            componentAreas[component] < minimumComponentArea) {
            continue;
        }
        const auto &indices = mesh.triangles()[faceIndex].vertexIndices;
        filtered.addTriangle(indices[0], indices[1], indices[2]);
    }
    return filtered;
}
#endif
}

ReconstructionMethod AlphaShapeReconstructor::method() const
{
    return ReconstructionMethod::AlphaShapes;
}

ReconstructionRequirements AlphaShapeReconstructor::requirements() const
{
    return {NormalRequirement::NotUsed, InputRepresentation::PointCloud};
}

ReconstructionResult AlphaShapeReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
#if !NEWELL_HAS_CGAL
    (void)input;
    (void)parameters;
    (void)progressCallback;
    (void)cancellationToken;
    result.errorMessage = "Alpha Shapes requires CGAL; configure with NEWELL_ENABLE_CGAL after installing CGAL.";
    return result;
#else
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing 2.5D Alpha Shapes");
    const auto *methodParameters = std::get_if<AlphaShapeParameters>(&parameters);
    if (!methodParameters || !input.pointCloud || input.pointCloud->pointCount() < 3U) {
        result.errorMessage = "Alpha Shapes requires matching parameters and at least three points.";
        return result;
    }
    if ((!methodParameters->automaticAlpha &&
         (!std::isfinite(methodParameters->alpha) || methodParameters->alpha <= 0.0)) ||
        !std::isfinite(methodParameters->automaticAlphaFactor) || methodParameters->automaticAlphaFactor <= 0.0 ||
        !std::isfinite(methodParameters->planarityTolerance) || methodParameters->planarityTolerance < 0.0 ||
        methodParameters->planarityTolerance >= 1.0 ||
        !std::isfinite(methodParameters->minimumComponentArea) || methodParameters->minimumComponentArea < 0.0 ||
        !std::isfinite(methodParameters->geometricTolerance) || methodParameters->geometricTolerance <= 0.0) {
        result.errorMessage =
            "Alpha Shapes requires positive alpha settings, a planarity tolerance in [0, 1), a non-negative component area, and a positive geometric tolerance.";
        return result;
    }
    const PlanarProjectionResult projectionResult = computePlanarProjection(
        *input.pointCloud, methodParameters->planarityTolerance);
    if (!projectionResult.succeeded) {
        result.errorMessage = projectionResult.errorMessage;
        return result;
    }
    const PlanarProjection &projection = projectionResult.projection;

    try {
        using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
        using InfoBase = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel>;
        using VertexBase = CGAL::Alpha_shape_vertex_base_2<Kernel, InfoBase>;
        using FaceBase = CGAL::Alpha_shape_face_base_2<Kernel>;
        using Structure = CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
        using Triangulation = CGAL::Delaunay_triangulation_2<Kernel, Structure>;
        using AlphaShape = CGAL::Alpha_shape_2<Triangulation>;
        Triangulation triangulation;
        std::map<std::pair<double, double>, std::size_t> uniqueProjectedPoints;
        for (std::size_t index = 0; index < input.pointCloud->pointCount(); ++index) {
            if ((index & 1023U) == 0U && cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            const Eigen::Vector2d &projected = projection.projectedPoints[index];
            if (!uniqueProjectedPoints.emplace(std::pair(projected.x(), projected.y()), index).second) {
                continue;
            }
            auto vertex = triangulation.insert(Kernel::Point_2(projected.x(), projected.y()));
            vertex->info() = index;
        }
        if (triangulation.dimension() != 2) {
            result.errorMessage = "Projected points are collinear or degenerate.";
            return result;
        }
        reporter.report(ProcessingStage::BuildingSpatialIndex, 0.58, "Built projected Delaunay structure");

        double effectiveAlpha = methodParameters->alpha;
        if (methodParameters->automaticAlpha) {
            std::map<std::size_t, double> nearestSquaredDistances;
            for (const auto &[position, index] : uniqueProjectedPoints) {
                (void)position;
                nearestSquaredDistances[index] = std::numeric_limits<double>::infinity();
            }
            for (auto edge = triangulation.finite_edges_begin(); edge != triangulation.finite_edges_end(); ++edge) {
                const auto face = edge->first;
                const int opposite = edge->second;
                const auto first = face->vertex((opposite + 1) % 3);
                const auto second = face->vertex((opposite + 2) % 3);
                const Eigen::Vector2d difference =
                    projection.projectedPoints[first->info()] - projection.projectedPoints[second->info()];
                const double squaredDistance = difference.squaredNorm();
                nearestSquaredDistances[first->info()] =
                    std::min(nearestSquaredDistances[first->info()], squaredDistance);
                nearestSquaredDistances[second->info()] =
                    std::min(nearestSquaredDistances[second->info()], squaredDistance);
            }
            std::vector<double> finiteDistances;
            finiteDistances.reserve(nearestSquaredDistances.size());
            for (const auto &[index, squaredDistance] : nearestSquaredDistances) {
                (void)index;
                if (std::isfinite(squaredDistance) && squaredDistance > 0.0) {
                    finiteDistances.push_back(squaredDistance);
                }
            }
            if (finiteDistances.empty()) {
                result.errorMessage = "Automatic alpha could not determine a valid projected point spacing.";
                return result;
            }
            const auto median = finiteDistances.begin() + static_cast<std::ptrdiff_t>(finiteDistances.size() / 2U);
            std::nth_element(finiteDistances.begin(), median, finiteDistances.end());
            effectiveAlpha = methodParameters->automaticAlphaFactor * *median;
        }
        if (!std::isfinite(effectiveAlpha) || effectiveAlpha <= 0.0) {
            result.errorMessage = "The effective alpha value is invalid.";
            return result;
        }

        AlphaShape alphaShape(
            triangulation,
            effectiveAlpha,
            methodParameters->regularised ? AlphaShape::REGULARIZED : AlphaShape::GENERAL);

        std::vector<Point3d> vertices;
        vertices.reserve(input.pointCloud->pointCount());
        for (std::size_t index = 0; index < input.pointCloud->pointCount(); ++index) {
            vertices.emplace_back(reconstructProjectedPoint(projection, index, methodParameters->mode));
        }
        TriangleMesh mesh;
        mesh.setVertices(std::move(vertices));
        for (auto face = alphaShape.finite_faces_begin(); face != alphaShape.finite_faces_end(); ++face) {
            if (cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            if (alphaShape.classify(face) != AlphaShape::INTERIOR) {
                continue;
            }
            std::array<std::size_t, 3> indices{
                face->vertex(0)->info(), face->vertex(1)->info(), face->vertex(2)->info()};
            const Eigen::Vector3d faceNormal =
                (mesh.vertices()[indices[1]].vector() - mesh.vertices()[indices[0]].vector())
                    .cross(mesh.vertices()[indices[2]].vector() - mesh.vertices()[indices[0]].vector());
            if (faceNormal.dot(projection.basis.normal) < 0.0) {
                std::swap(indices[1], indices[2]);
            }
            mesh.addTriangle(indices[0], indices[1], indices[2]);
        }
        mesh = filterMeshComponents(
            mesh,
            methodParameters->keepLargestComponentOnly,
            methodParameters->minimumComponentArea);
        MeshValidationResult validation = validateMesh(std::move(mesh), methodParameters->geometricTolerance);
        if (!validation.valid) {
            result.errorMessage = validation.errorMessage.empty()
                ? "Alpha Shapes generated no valid triangles."
                : std::move(validation.errorMessage);
            return result;
        }
        result.mesh = std::move(validation.mesh);
        result.diagnostics = validation.diagnostics;
        result.succeeded = true;
        reporter.report(ProcessingStage::Finalizing, 0.99, "Alpha Shape surface ready");
        return result;
    } catch (const std::exception &exception) {
        result.errorMessage = std::string("CGAL Alpha Shapes failed: ") + exception.what();
        return result;
    } catch (...) {
        result.errorMessage = "CGAL Alpha Shapes failed with an unknown exception.";
        return result;
    }
#endif
}
