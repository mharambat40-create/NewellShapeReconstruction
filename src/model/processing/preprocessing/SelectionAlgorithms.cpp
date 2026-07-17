#include "model/processing/preprocessing/SelectionAlgorithms.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/spatial/KDTree.h"

#include <map>
#include <tuple>

namespace
{
constexpr std::size_t kProgressBlockSize = 128U;

bool reportPointProgress(
    ProgressReporter &reporter,
    ProcessingStage stage,
    std::size_t processedPointCount,
    std::size_t totalPointCount,
    const char *message)
{
    if (processedPointCount % kProgressBlockSize != 0U && processedPointCount != totalPointCount) {
        return false;
    }
    reporter.report(
        stage,
        totalPointCount == 0U
            ? 1.0
            : static_cast<double>(processedPointCount) / static_cast<double>(totalPointCount),
        message);
    return true;
}

PointSelectionResult cancelledResult(ProgressReporter &reporter)
{
    reporter.cancelled("Processing cancelled.");
    return PointSelectionResult{false, true, {}, "Processing cancelled."};
}
}

PointSelectionResult InvalidPointSelector::selectSparsePoints(
    const PointCloud &pointCloud,
    double radius,
    int minimumNeighbourCount,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ProgressReporter reporter(progressCallback);
    if (pointCloud.empty()) {
        return PointSelectionResult{false, false, {}, "No point cloud is available."};
    }
    if (radius <= 0.0 || minimumNeighbourCount < 0) {
        return PointSelectionResult{false, false, {}, "Sparse-point parameters are invalid."};
    }

    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.0, "Building neighbourhood index");
    const KDTree neighbourSearch(pointCloud);
    if (cancellationToken.isCancellationRequested()) {
        return cancelledResult(reporter);
    }

    PointSelectionResult result;
    result.selectedIndices.reserve(pointCloud.pointCount() / 20U);
    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            return cancelledResult(reporter);
        }

        const auto neighbours = neighbourSearch.radiusSearch(
            pointCloud.points()[pointIndex].vector(),
            radius,
            pointIndex);
        if (neighbours.size() < static_cast<std::size_t>(minimumNeighbourCount)) {
            result.selectedIndices.push_back(pointIndex);
        }
        reportPointProgress(
            reporter,
            ProcessingStage::SelectingInvalidPoints,
            pointIndex + 1U,
            pointCloud.pointCount(),
            "Analysing point neighbourhoods");
    }

    result.succeeded = true;
    reporter.complete("Invalid-point selection complete");
    return result;
}

PointSelectionResult DuplicatePointSelector::selectPerfectDuplicates(
    const PointCloud &pointCloud,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ProgressReporter reporter(progressCallback);
    if (pointCloud.empty()) {
        return PointSelectionResult{false, false, {}, "No point cloud is available."};
    }

    using CoordinateKey = std::tuple<double, double, double>;
    std::map<CoordinateKey, std::size_t> representatives;
    PointSelectionResult result;
    result.selectedIndices.reserve(pointCloud.pointCount() / 2U);

    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            return cancelledResult(reporter);
        }

        const Point3d &point = pointCloud.points()[pointIndex];
        if (!representatives.emplace(
                CoordinateKey(point.x(), point.y(), point.z()),
                pointIndex).second) {
            result.selectedIndices.push_back(pointIndex);
        }
        reportPointProgress(
            reporter,
            ProcessingStage::SelectingDuplicates,
            pointIndex + 1U,
            pointCloud.pointCount(),
            "Finding exact duplicates");
    }

    result.succeeded = true;
    reporter.complete("Duplicate selection complete");
    return result;
}

PointSelectionResult DuplicatePointSelector::selectNearDuplicates(
    const PointCloud &pointCloud,
    double distanceThreshold,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ProgressReporter reporter(progressCallback);
    if (pointCloud.empty()) {
        return PointSelectionResult{false, false, {}, "No point cloud is available."};
    }
    if (distanceThreshold <= 0.0) {
        return PointSelectionResult{false, false, {}, "Distance threshold must be greater than zero."};
    }

    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.0, "Building neighbourhood index");
    const KDTree neighbourSearch(pointCloud);
    if (cancellationToken.isCancellationRequested()) {
        return cancelledResult(reporter);
    }

    PointSelectionResult result;
    result.selectedIndices.reserve(pointCloud.pointCount() / 2U);
    std::vector<bool> representatives(pointCloud.pointCount(), false);
    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            return cancelledResult(reporter);
        }

        bool matchesRepresentative = false;
        for (const auto &neighbour : neighbourSearch.radiusSearch(
                 pointCloud.points()[pointIndex].vector(),
                 distanceThreshold,
                 pointIndex)) {
            if (neighbour.pointIndex < pointIndex && representatives[neighbour.pointIndex]) {
                matchesRepresentative = true;
                break;
            }
        }

        if (matchesRepresentative) {
            result.selectedIndices.push_back(pointIndex);
        } else {
            representatives[pointIndex] = true;
        }
        reportPointProgress(
            reporter,
            ProcessingStage::SelectingDuplicates,
            pointIndex + 1U,
            pointCloud.pointCount(),
            "Grouping nearby points");
    }

    result.succeeded = true;
    reporter.complete("Duplicate selection complete");
    return result;
}
