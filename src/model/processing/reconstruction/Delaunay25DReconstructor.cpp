#include "model/processing/reconstruction/Delaunay25DReconstructor.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/reconstruction/common/ProjectionPlaneUtilities.h"

#include <Eigen/Geometry>

#include <cmath>
#include <exception>
#include <map>
#include <string>
#include <utility>
#include <vector>

#if NEWELL_HAS_CGAL
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#endif

ReconstructionMethod Delaunay25DReconstructor::method() const
{
    return ReconstructionMethod::Delaunay25D;
}

ReconstructionRequirements Delaunay25DReconstructor::requirements() const
{
    return {NormalRequirement::NotUsed, InputRepresentation::PointCloud};
}

ReconstructionResult Delaunay25DReconstructor::reconstruct(
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
    result.errorMessage = "Delaunay 2.5D requires CGAL; configure with NEWELL_ENABLE_CGAL after installing CGAL.";
    return result;
#else
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing Delaunay 2.5D");
    const auto *methodParameters = std::get_if<Delaunay25DParameters>(&parameters);
    if (!methodParameters || !input.pointCloud || input.pointCloud->pointCount() < 3U) {
        result.errorMessage = "Delaunay 2.5D requires matching parameters and at least three points.";
        return result;
    }
    if (!std::isfinite(methodParameters->maximumEdgeLength) || methodParameters->maximumEdgeLength < 0.0 ||
        !std::isfinite(methodParameters->planarityTolerance) || methodParameters->planarityTolerance < 0.0 ||
        methodParameters->planarityTolerance >= 1.0 ||
        !std::isfinite(methodParameters->geometricTolerance) || methodParameters->geometricTolerance <= 0.0) {
        result.errorMessage =
            "Delaunay parameters require a non-negative edge limit, a planarity tolerance in [0, 1), and a positive geometric tolerance.";
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
        using VertexBase = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel>;
        using FaceBase = CGAL::Triangulation_face_base_2<Kernel>;
        using Structure = CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>;
        using Triangulation = CGAL::Delaunay_triangulation_2<Kernel, Structure>;
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
        reporter.report(ProcessingStage::BuildingSpatialIndex, 0.65, "Built projected Delaunay structure");
        if (triangulation.dimension() != 2) {
            result.errorMessage = "Projected points are collinear or degenerate.";
            return result;
        }

        std::vector<Point3d> vertices;
        vertices.reserve(input.pointCloud->pointCount());
        for (std::size_t index = 0; index < input.pointCloud->pointCount(); ++index) {
            vertices.emplace_back(reconstructProjectedPoint(projection, index, methodParameters->mode));
        }

        TriangleMesh mesh;
        mesh.setVertices(std::move(vertices));
        for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
            if (cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            std::array<std::size_t, 3> indices{
                face->vertex(0)->info(), face->vertex(1)->info(), face->vertex(2)->info()};
            if (methodParameters->maximumEdgeLength > 0.0) {
                bool longEdge = false;
                for (std::size_t edge = 0; edge < 3U; ++edge) {
                    const Eigen::Vector3d difference =
                        mesh.vertices()[indices[edge]].vector() -
                        mesh.vertices()[indices[(edge + 1U) % 3U]].vector();
                    longEdge = longEdge || difference.norm() > methodParameters->maximumEdgeLength;
                }
                if (longEdge) {
                    continue;
                }
            }
            const Eigen::Vector3d faceNormal =
                (mesh.vertices()[indices[1]].vector() - mesh.vertices()[indices[0]].vector())
                    .cross(mesh.vertices()[indices[2]].vector() - mesh.vertices()[indices[0]].vector());
            if (faceNormal.dot(projection.basis.normal) < 0.0) {
                std::swap(indices[1], indices[2]);
            }
            mesh.addTriangle(indices[0], indices[1], indices[2]);
        }
        MeshValidationResult validation = validateMesh(std::move(mesh), methodParameters->geometricTolerance);
        if (!validation.valid) {
            result.errorMessage = validation.errorMessage.empty()
                ? "Delaunay generated no valid triangles."
                : std::move(validation.errorMessage);
            return result;
        }
        result.mesh = std::move(validation.mesh);
        result.diagnostics = validation.diagnostics;
        result.succeeded = true;
        reporter.report(ProcessingStage::Finalizing, 0.99, "Delaunay surface ready");
        return result;
    } catch (const std::exception &exception) {
        result.errorMessage = std::string("CGAL Delaunay failed: ") + exception.what();
        return result;
    } catch (...) {
        result.errorMessage = "CGAL Delaunay failed with an unknown exception.";
        return result;
    }
#endif
}
