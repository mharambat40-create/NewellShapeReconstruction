#include "model/processing/reconstruction/BallPivotingReconstructor.h"

#include "model/geometry/NormalField.h"
#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <compare>
#include <deque>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace
{
constexpr std::size_t kMaximumSeedNeighbours = 32U;
constexpr std::size_t kMaximumPivotCandidates = 96U;
constexpr double kMinimumNormalAlignment = 1.0e-3;
constexpr std::size_t kProgressEdgeBlockSize = 64U;

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point &start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct EdgeKey
{
    std::size_t first = 0U;
    std::size_t second = 0U;
    auto operator<=>(const EdgeKey &) const = default;
};

struct TriangleKey
{
    std::array<std::size_t, 3> indices{};
    auto operator<=>(const TriangleKey &) const = default;
};

struct FrontierEdge
{
    std::size_t from = 0U;
    std::size_t to = 0U;
    Eigen::Vector3d ballCenter = Eigen::Vector3d::Zero();
};

struct SeedTriangle
{
    std::array<std::size_t, 3> indices{};
    Eigen::Vector3d ballCenter = Eigen::Vector3d::Zero();
};

EdgeKey edgeKey(std::size_t first, std::size_t second)
{
    return EdgeKey{std::min(first, second), std::max(first, second)};
}

TriangleKey triangleKey(std::size_t first, std::size_t second, std::size_t third)
{
    std::array<std::size_t, 3> indices{first, second, third};
    std::sort(indices.begin(), indices.end());
    return TriangleKey{indices};
}

std::vector<Eigen::Vector3d> ballCentersForTriangle(
    const Eigen::Vector3d &first,
    const Eigen::Vector3d &second,
    const Eigen::Vector3d &third,
    double ballRadius,
    double tolerance)
{
    const Eigen::Vector3d firstEdge = second - first;
    const Eigen::Vector3d secondEdge = third - first;
    const Eigen::Vector3d cross = firstEdge.cross(secondEdge);
    const double crossSquaredNorm = cross.squaredNorm();
    if (crossSquaredNorm <= tolerance * tolerance) {
        return {};
    }

    const Eigen::Vector3d circumcenter = first +
        (firstEdge.squaredNorm() * secondEdge.cross(cross) +
         secondEdge.squaredNorm() * cross.cross(firstEdge)) /
            (2.0 * crossSquaredNorm);
    const double circumradiusSquared = (circumcenter - first).squaredNorm();
    const double ballRadiusSquared = ballRadius * ballRadius;
    if (circumradiusSquared > ballRadiusSquared + tolerance) {
        return {};
    }

    const double height = std::sqrt(std::max(0.0, ballRadiusSquared - circumradiusSquared));
    const Eigen::Vector3d planeNormal = cross.normalized();
    if (height <= tolerance) {
        return {circumcenter};
    }
    return {circumcenter + height * planeNormal, circumcenter - height * planeNormal};
}

bool ballIsEmpty(
    const Eigen::Vector3d &center,
    const std::array<std::size_t, 3> &triangle,
    const INeighbourSearch &neighbourSearch,
    double ballRadius,
    double tolerance)
{
    const double interiorRadius = std::max(0.0, ballRadius - tolerance);
    for (const Neighbour &neighbour : neighbourSearch.radiusSearch(center, interiorRadius)) {
        if (neighbour.pointIndex != triangle[0] &&
            neighbour.pointIndex != triangle[1] &&
            neighbour.pointIndex != triangle[2]) {
            return false;
        }
    }
    return true;
}

bool normalsSupportBall(
    const std::array<std::size_t, 3> &triangle,
    const Eigen::Vector3d &center,
    const PointCloud &pointCloud,
    const NormalField &normalField)
{
    for (const std::size_t pointIndex : triangle) {
        const Normal3d &normal = normalField.normal(pointIndex);
        if (!normal.isValid()) {
            return false;
        }
        const Eigen::Vector3d contactDirection = center - pointCloud.points()[pointIndex].vector();
        if (contactDirection.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        // PCA supplies unoriented directions in this slice, so compatibility is sign-independent.
        if (std::abs(normal.vector().dot(contactDirection.normalized())) < kMinimumNormalAlignment) {
            return false;
        }
    }
    return true;
}

double positivePivotAngle(
    const Eigen::Vector3d &edgeStart,
    const Eigen::Vector3d &edgeEnd,
    const Eigen::Vector3d &currentCenter,
    const Eigen::Vector3d &candidateCenter,
    double tolerance)
{
    const Eigen::Vector3d edge = edgeEnd - edgeStart;
    if (edge.squaredNorm() <= tolerance * tolerance) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector3d axis = edge.normalized();
    const Eigen::Vector3d midpoint = 0.5 * (edgeStart + edgeEnd);
    Eigen::Vector3d currentRadial = currentCenter - midpoint;
    Eigen::Vector3d candidateRadial = candidateCenter - midpoint;
    currentRadial -= axis * currentRadial.dot(axis);
    candidateRadial -= axis * candidateRadial.dot(axis);
    if (currentRadial.squaredNorm() <= tolerance * tolerance ||
        candidateRadial.squaredNorm() <= tolerance * tolerance) {
        return std::numeric_limits<double>::infinity();
    }

    currentRadial.normalize();
    candidateRadial.normalize();
    double angle = std::atan2(
        axis.dot(currentRadial.cross(candidateRadial)),
        std::clamp(currentRadial.dot(candidateRadial), -1.0, 1.0));
    if (angle <= tolerance) {
        angle += 2.0 * std::numbers::pi_v<double>;
    }
    return angle;
}
}

ReconstructionMethod BallPivotingReconstructor::method() const
{
    return ReconstructionMethod::BallPivoting;
}

ReconstructionRequirements BallPivotingReconstructor::requirements() const
{
    return {NormalRequirement::Required, InputRepresentation::PointCloud};
}

ReconstructionResult BallPivotingReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    const auto *ballParameters = std::get_if<BallPivotingParameters>(&parameters);
    if (!ballParameters) {
        ReconstructionResult result;
        result.errorMessage = "Ball Pivoting received parameters for another reconstruction method.";
        return result;
    }
    if (!input.pointCloud || !input.normalField || !input.neighbourSearch) {
        ReconstructionResult result;
        result.errorMessage = "Ball Pivoting requires a point cloud, normals and a neighbour index.";
        return result;
    }
    return reconstruct(
        *input.pointCloud,
        *input.normalField,
        *input.neighbourSearch,
        *ballParameters,
        progressCallback,
        cancellationToken);
}

ReconstructionResult BallPivotingReconstructor::reconstruct(
    const PointCloud &pointCloud,
    const NormalField &normalField,
    const INeighbourSearch &neighbourSearch,
    const ReconstructionParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
    ProgressReporter reporter(progressCallback);
    const auto preparationStart = Clock::now();
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing Ball Pivoting");
    if (pointCloud.pointCount() < 3U) {
        result.errorMessage = "At least three points are required for Ball Pivoting reconstruction.";
        return result;
    }
    if (normalField.size() != pointCloud.pointCount()) {
        result.errorMessage = "The normal field must contain one entry per point.";
        return result;
    }
    if (parameters.ballRadius <= 0.0 || parameters.geometricTolerance <= 0.0) {
        result.errorMessage = "Ball radius and geometric tolerance must be positive.";
        return result;
    }

    if (cancellationToken.isCancellationRequested()) {
        result.cancelled = true;
        result.errorMessage = "Processing cancelled.";
        reporter.cancelled(result.errorMessage);
        return result;
    }
    reporter.report(ProcessingStage::Preparing, 0.03, "Ball Pivoting input validated");

    const double tolerance = std::max(
        parameters.geometricTolerance,
        parameters.ballRadius * 1.0e-9);
    result.mesh.setVertices(pointCloud.points());

    std::set<TriangleKey> triangles;
    std::map<EdgeKey, std::size_t> edgeIncidence;
    std::map<EdgeKey, FrontierEdge> frontier;
    std::deque<EdgeKey> frontierQueue;
    std::vector<bool> usedVertices(pointCloud.pointCount(), false);
    std::size_t processedFrontEdges = 0U;
    std::size_t usedVertexCount = 0U;
    std::size_t maximumActiveFrontEdges = 0U;
    std::size_t createdTriangles = 0U;
    std::size_t rejectedCandidates = 0U;
    const std::size_t eligiblePointCount = std::count_if(
        normalField.normals().begin(),
        normalField.normals().end(),
        [](const Normal3d &normal) { return normal.isValid(); });
    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.07, "Preparing Ball Pivoting topology");
    result.timings.preparationMilliseconds = elapsedMilliseconds(preparationStart);

    const auto canAddTriangle = [&](const std::array<std::size_t, 3> &indices) {
        if (triangles.contains(triangleKey(indices[0], indices[1], indices[2]))) {
            return false;
        }
        for (const EdgeKey &key : {
                 edgeKey(indices[0], indices[1]),
                 edgeKey(indices[1], indices[2]),
                 edgeKey(indices[2], indices[0])}) {
            const auto existing = edgeIncidence.find(key);
            if (existing != edgeIncidence.end() && existing->second >= 2U) {
                return false;
            }
        }
        return true;
    };

    const auto addTriangle = [&](const std::array<std::size_t, 3> &indices,
                                 const Eigen::Vector3d &ballCenter) {
        if (!canAddTriangle(indices)) {
            return false;
        }

        triangles.insert(triangleKey(indices[0], indices[1], indices[2]));
        result.mesh.addTriangle(indices[0], indices[1], indices[2]);
        for (const std::size_t index : indices) {
            if (!usedVertices[index]) {
                ++usedVertexCount;
            }
            usedVertices[index] = true;
        }
        ++createdTriangles;

        const std::array<std::array<std::size_t, 2>, 3> directedEdges{{
            {indices[0], indices[1]},
            {indices[1], indices[2]},
            {indices[2], indices[0]},
        }};
        for (const auto &directedEdge : directedEdges) {
            const EdgeKey key = edgeKey(directedEdge[0], directedEdge[1]);
            std::size_t &incidence = edgeIncidence[key];
            ++incidence;
            if (incidence == 1U) {
                frontier[key] = FrontierEdge{directedEdge[0], directedEdge[1], ballCenter};
                frontierQueue.push_back(key);
            } else {
                frontier.erase(key);
            }
        }
        maximumActiveFrontEdges = std::max(maximumActiveFrontEdges, frontier.size());
        return true;
    };

    const auto findSeed = [&]() -> std::optional<SeedTriangle> {
        for (std::size_t first = 0; first < pointCloud.pointCount(); ++first) {
            if (cancellationToken.isCancellationRequested()) {
                return std::nullopt;
            }
            reporter.report(
                ProcessingStage::FindingSeed,
                mapPhaseProgress(
                    0.07,
                    0.12,
                    static_cast<double>(first + 1U) / static_cast<double>(pointCloud.pointCount())),
                "Searching for a seed triangle");
            if (usedVertices[first] || !normalField.normal(first).isValid()) {
                continue;
            }
            std::vector<Neighbour> neighbours = neighbourSearch.radiusSearch(
                pointCloud.points()[first].vector(),
                2.0 * parameters.ballRadius,
                first);
            if (neighbours.size() > kMaximumSeedNeighbours) {
                neighbours.resize(kMaximumSeedNeighbours);
            }

            for (std::size_t secondCursor = 0; secondCursor < neighbours.size(); ++secondCursor) {
                const std::size_t second = neighbours[secondCursor].pointIndex;
                if (usedVertices[second] || !normalField.normal(second).isValid()) {
                    continue;
                }
                for (std::size_t thirdCursor = secondCursor + 1U; thirdCursor < neighbours.size(); ++thirdCursor) {
                    if (cancellationToken.isCancellationRequested()) {
                        return std::nullopt;
                    }
                    const std::size_t third = neighbours[thirdCursor].pointIndex;
                    if (usedVertices[third] || !normalField.normal(third).isValid()) {
                        continue;
                    }

                    std::array<std::size_t, 3> indices{first, second, third};
                    const auto centers = ballCentersForTriangle(
                        pointCloud.points()[first].vector(),
                        pointCloud.points()[second].vector(),
                        pointCloud.points()[third].vector(),
                        parameters.ballRadius,
                        tolerance);
                    for (const Eigen::Vector3d &center : centers) {
                        if (!ballIsEmpty(center, indices, neighbourSearch, parameters.ballRadius, tolerance) ||
                            !normalsSupportBall(indices, center, pointCloud, normalField)) {
                            continue;
                        }

                        const Eigen::Vector3d faceNormal =
                            (pointCloud.points()[second].vector() - pointCloud.points()[first].vector())
                                .cross(pointCloud.points()[third].vector() - pointCloud.points()[first].vector());
                        const Eigen::Vector3d centroid =
                            (pointCloud.points()[first].vector() +
                             pointCloud.points()[second].vector() +
                             pointCloud.points()[third].vector()) /
                            3.0;
                        if (faceNormal.dot(center - centroid) < 0.0) {
                            std::swap(indices[1], indices[2]);
                        }
                        return SeedTriangle{indices, center};
                    }
                }
            }
        }
        return std::nullopt;
    };

    while (true) {
        const auto seedSearchStart = Clock::now();
        const std::optional<SeedTriangle> seed = findSeed();
        result.timings.seedSearchMilliseconds += elapsedMilliseconds(seedSearchStart);
        if (!seed) {
            break;
        }
        if (cancellationToken.isCancellationRequested()) {
            break;
        }
        addTriangle(seed->indices, seed->ballCenter);

        const auto propagationStart = Clock::now();
        while (!frontierQueue.empty()) {
            if (cancellationToken.isCancellationRequested()) {
                break;
            }
            const EdgeKey key = frontierQueue.front();
            frontierQueue.pop_front();
            ++processedFrontEdges;
            const std::size_t activeFrontEdges = frontier.size();
            maximumActiveFrontEdges = std::max(maximumActiveFrontEdges, activeFrontEdges);
            const double meshedPointFraction = eligiblePointCount == 0U
                ? 0.0
                : static_cast<double>(usedVertexCount) / static_cast<double>(eligiblePointCount);
            const double remainingEligiblePoints = static_cast<double>(
                eligiblePointCount > usedVertexCount ? eligiblePointCount - usedVertexCount : 0U);
            const double estimatedFutureWork = std::max(
                0.5 * remainingEligiblePoints,
                0.25 * static_cast<double>(maximumActiveFrontEdges));
            const double frontCompletionEstimate = static_cast<double>(processedFrontEdges) /
                (static_cast<double>(processedFrontEdges + activeFrontEdges) + estimatedFutureWork);
            const double propagationFraction = std::clamp(
                0.65 * meshedPointFraction + 0.35 * frontCompletionEstimate,
                0.0,
                0.99);
            if (processedFrontEdges % kProgressEdgeBlockSize == 0U || frontierQueue.empty()) {
                reporter.report(
                    ProcessingStage::PropagatingSurface,
                    mapPhaseProgress(0.12, 0.94, propagationFraction),
                    "Reconstructing surface");
            }
            const auto activeEdge = frontier.find(key);
            if (activeEdge == frontier.end()) {
                continue;
            }
            const FrontierEdge edge = activeEdge->second;
            const Eigen::Vector3d &edgeStart = pointCloud.points()[edge.from].vector();
            const Eigen::Vector3d &edgeEnd = pointCloud.points()[edge.to].vector();
            const Eigen::Vector3d midpoint = 0.5 * (edgeStart + edgeEnd);
            std::vector<Neighbour> candidates = neighbourSearch.radiusSearch(
                midpoint,
                2.0 * parameters.ballRadius);
            if (candidates.size() > kMaximumPivotCandidates) {
                candidates.resize(kMaximumPivotCandidates);
            }

            double bestAngle = std::numeric_limits<double>::infinity();
            std::optional<std::array<std::size_t, 3>> bestTriangle;
            Eigen::Vector3d bestCenter = Eigen::Vector3d::Zero();
            for (const Neighbour &candidate : candidates) {
                if (cancellationToken.isCancellationRequested()) {
                    break;
                }
                const std::size_t third = candidate.pointIndex;
                if (third == edge.from || third == edge.to || !normalField.normal(third).isValid()) {
                    ++rejectedCandidates;
                    continue;
                }
                if ((pointCloud.points()[third].vector() - edgeStart).norm() > 2.0 * parameters.ballRadius + tolerance ||
                    (pointCloud.points()[third].vector() - edgeEnd).norm() > 2.0 * parameters.ballRadius + tolerance) {
                    ++rejectedCandidates;
                    continue;
                }

                const std::array<std::size_t, 3> indices{edge.to, edge.from, third};
                if (!canAddTriangle(indices)) {
                    ++rejectedCandidates;
                    continue;
                }
                const auto centers = ballCentersForTriangle(
                    pointCloud.points()[indices[0]].vector(),
                    pointCloud.points()[indices[1]].vector(),
                    pointCloud.points()[indices[2]].vector(),
                    parameters.ballRadius,
                    tolerance);
                for (const Eigen::Vector3d &center : centers) {
                    if (!ballIsEmpty(center, indices, neighbourSearch, parameters.ballRadius, tolerance) ||
                        !normalsSupportBall(indices, center, pointCloud, normalField)) {
                        continue;
                    }
                    const double angle = positivePivotAngle(
                        edgeStart,
                        edgeEnd,
                        edge.ballCenter,
                        center,
                        tolerance);
                    if (angle < bestAngle) {
                        bestAngle = angle;
                        bestTriangle = indices;
                        bestCenter = center;
                    }
                }
            }

            if (bestTriangle) {
                addTriangle(*bestTriangle, bestCenter);
            } else {
                frontier.erase(key);
            }
        }
        result.timings.frontPropagationMilliseconds += elapsedMilliseconds(propagationStart);
    }

    if (cancellationToken.isCancellationRequested()) {
        result.mesh.clear();
        result.cancelled = true;
        result.errorMessage = "Processing cancelled.";
        reporter.cancelled(result.errorMessage);
        return result;
    }

    const auto meshValidationStart = Clock::now();
    reporter.report(ProcessingStage::Finalizing, 0.94, "Validating reconstructed surface");
    if (result.mesh.empty()) {
        result.errorMessage =
            "Ball Pivoting could not find a valid empty-ball seed. Check the ball radius and normal neighbourhood.";
        result.timings.meshValidationMilliseconds = elapsedMilliseconds(meshValidationStart);
        return result;
    }
    if (!result.mesh.hasValidIndices()) {
        result.mesh.clear();
        result.errorMessage = "Ball Pivoting produced invalid mesh indices.";
        result.timings.meshValidationMilliseconds = elapsedMilliseconds(meshValidationStart);
        return result;
    }

    result.succeeded = true;
    result.timings.meshValidationMilliseconds = elapsedMilliseconds(meshValidationStart);
    reporter.report(
        ProcessingStage::Finalizing,
        0.99,
        "Ball Pivoting validation complete (" + std::to_string(createdTriangles) +
            " triangles, " + std::to_string(rejectedCandidates) + " rejected candidates)");
    return result;
}
