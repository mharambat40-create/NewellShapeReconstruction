#include "model/processing/reconstruction/GreedyProjectionReconstructor.h"

#include "model/geometry/NormalField.h"
#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <compare>
#include <map>
#include <numbers>
#include <set>
#include <utility>
#include <vector>

namespace
{
struct FaceKey
{
    std::array<std::size_t, 3> indices{};
    auto operator<=>(const FaceKey &) const = default;
};

struct EdgeKey
{
    std::size_t first = 0U;
    std::size_t second = 0U;
    auto operator<=>(const EdgeKey &) const = default;
};

FaceKey faceKey(std::array<std::size_t, 3> indices)
{
    std::sort(indices.begin(), indices.end());
    return {indices};
}

EdgeKey edgeKey(std::size_t first, std::size_t second)
{
    return {std::min(first, second), std::max(first, second)};
}

std::array<double, 3> triangleAngles(
    const Eigen::Vector3d &first,
    const Eigen::Vector3d &second,
    const Eigen::Vector3d &third)
{
    const double firstLength = (second - third).norm();
    const double secondLength = (first - third).norm();
    const double thirdLength = (first - second).norm();
    const auto angle = [](double opposite, double adjacentFirst, double adjacentSecond) {
        const double denominator = 2.0 * adjacentFirst * adjacentSecond;
        if (denominator <= 0.0) {
            return 0.0;
        }
        return std::acos(std::clamp(
            (adjacentFirst * adjacentFirst + adjacentSecond * adjacentSecond - opposite * opposite) /
                denominator,
            -1.0,
            1.0));
    };
    return {
        angle(firstLength, secondLength, thirdLength),
        angle(secondLength, firstLength, thirdLength),
        angle(thirdLength, firstLength, secondLength),
    };
}
}

ReconstructionMethod GreedyProjectionReconstructor::method() const
{
    return ReconstructionMethod::GreedyProjection;
}

ReconstructionRequirements GreedyProjectionReconstructor::requirements() const
{
    return {NormalRequirement::Required, InputRepresentation::PointCloud};
}

ReconstructionResult GreedyProjectionReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing Greedy Projection");
    const auto *methodParameters = std::get_if<GreedyProjectionParameters>(&parameters);
    if (!methodParameters || !input.pointCloud || !input.normalField || !input.neighbourSearch) {
        result.errorMessage = "Greedy Projection requires matching parameters, points, normals and a neighbour index.";
        return result;
    }
    if (input.pointCloud->pointCount() < 3U ||
        input.normalField->size() != input.pointCloud->pointCount()) {
        result.errorMessage = "Greedy Projection requires at least three points and one normal per point.";
        return result;
    }
    if (!std::isfinite(methodParameters->searchRadius) || methodParameters->searchRadius <= 0.0 ||
        methodParameters->maximumNeighbours < 2U ||
        methodParameters->minimumTriangleAngleRadians <= 0.0 ||
        methodParameters->maximumTriangleAngleRadians >= std::numbers::pi ||
        methodParameters->minimumTriangleAngleRadians >= methodParameters->maximumTriangleAngleRadians ||
        methodParameters->maximumSurfaceAngleRadians <= 0.0 ||
        methodParameters->maximumSurfaceAngleRadians > std::numbers::pi) {
        result.errorMessage = "Greedy Projection parameters are outside their valid ranges.";
        return result;
    }

    TriangleMesh mesh;
    mesh.setVertices(input.pointCloud->points());
    std::set<FaceKey> faces;
    std::map<EdgeKey, std::size_t> edgeIncidence;
    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.05, "Using shared neighbour index");

    for (std::size_t seed = 0; seed < input.pointCloud->pointCount(); ++seed) {
        if ((seed & 63U) == 0U && cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        const Normal3d &seedNormal = input.normalField->normal(seed);
        if (!seedNormal.isValid()) {
            continue;
        }
        std::vector<Neighbour> neighbours = input.neighbourSearch->radiusSearch(
            input.pointCloud->points()[seed].vector(), methodParameters->searchRadius, seed);
        if (neighbours.size() > methodParameters->maximumNeighbours) {
            neighbours.resize(methodParameters->maximumNeighbours);
        }
        neighbours.erase(
            std::remove_if(
                neighbours.begin(),
                neighbours.end(),
                [&](const Neighbour &neighbour) {
                    const Normal3d &normal = input.normalField->normal(neighbour.pointIndex);
                    return !normal.isValid() ||
                        std::abs(seedNormal.vector().dot(normal.vector())) <
                            std::cos(methodParameters->maximumSurfaceAngleRadians);
                }),
            neighbours.end());
        if (neighbours.size() < 2U) {
            continue;
        }

        const Eigen::Vector3d normal = seedNormal.vector();
        const Eigen::Vector3d reference = std::abs(normal.z()) < 0.9
            ? Eigen::Vector3d::UnitZ()
            : Eigen::Vector3d::UnitX();
        const Eigen::Vector3d tangent = normal.cross(reference).normalized();
        const Eigen::Vector3d bitangent = normal.cross(tangent).normalized();
        const Eigen::Vector3d &origin = input.pointCloud->points()[seed].vector();
        std::sort(neighbours.begin(), neighbours.end(), [&](const Neighbour &left, const Neighbour &right) {
            const Eigen::Vector3d leftOffset = input.pointCloud->points()[left.pointIndex].vector() - origin;
            const Eigen::Vector3d rightOffset = input.pointCloud->points()[right.pointIndex].vector() - origin;
            const double leftAngle = std::atan2(leftOffset.dot(bitangent), leftOffset.dot(tangent));
            const double rightAngle = std::atan2(rightOffset.dot(bitangent), rightOffset.dot(tangent));
            return leftAngle < rightAngle;
        });

        for (std::size_t cursor = 0; cursor < neighbours.size(); ++cursor) {
            std::array<std::size_t, 3> indices{
                seed,
                neighbours[cursor].pointIndex,
                neighbours[(cursor + 1U) % neighbours.size()].pointIndex,
            };
            if (faceKey(indices).indices[0] == faceKey(indices).indices[1]) {
                continue;
            }
            const Eigen::Vector3d &first = input.pointCloud->points()[indices[0]].vector();
            const Eigen::Vector3d &second = input.pointCloud->points()[indices[1]].vector();
            const Eigen::Vector3d &third = input.pointCloud->points()[indices[2]].vector();
            const Eigen::Vector3d faceNormal = (second - first).cross(third - first);
            if (faceNormal.norm() <= methodParameters->geometricTolerance) {
                continue;
            }
            const std::array<double, 3> angles = triangleAngles(first, second, third);
            if (std::ranges::any_of(angles, [&](double angle) {
                    return angle < methodParameters->minimumTriangleAngleRadians ||
                        angle > methodParameters->maximumTriangleAngleRadians;
                })) {
                continue;
            }
            const std::array<EdgeKey, 3> edges{
                edgeKey(indices[0], indices[1]),
                edgeKey(indices[1], indices[2]),
                edgeKey(indices[2], indices[0]),
            };
            if (std::ranges::any_of(edges, [&](const EdgeKey &edge) {
                    const auto existing = edgeIncidence.find(edge);
                    return existing != edgeIncidence.end() && existing->second >= 2U;
                }) || !faces.insert(faceKey(indices)).second) {
                continue;
            }
            if (faceNormal.dot(normal) < 0.0) {
                std::swap(indices[1], indices[2]);
            }
            mesh.addTriangle(indices[0], indices[1], indices[2]);
            for (const EdgeKey &edge : edges) {
                ++edgeIncidence[edge];
            }
        }
        if ((seed & 31U) == 0U || seed + 1U == input.pointCloud->pointCount()) {
            reporter.report(
                ProcessingStage::PropagatingSurface,
                mapPhaseProgress(
                    0.05,
                    0.94,
                    static_cast<double>(seed + 1U) /
                        static_cast<double>(input.pointCloud->pointCount())),
                "Growing projected neighbourhood triangles");
        }
    }

    reporter.report(ProcessingStage::Finalizing, 0.95, "Validating Greedy Projection mesh");
    MeshValidationResult validation = validateMesh(
        std::move(mesh), methodParameters->geometricTolerance);
    if (!validation.valid) {
        result.errorMessage = std::move(validation.errorMessage);
        return result;
    }
    result.mesh = std::move(validation.mesh);
    result.diagnostics = validation.diagnostics;
    result.succeeded = true;
    reporter.report(ProcessingStage::Finalizing, 0.99, "Greedy Projection mesh ready");
    return result;
}
