#include "model/processing/reconstruction/VoxelReconstructor.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/reconstruction/MarchingCubesReconstructor.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace
{
const std::array<Eigen::Vector3i, 7> kGapClosingOffsets{{
    {0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
}};

std::optional<Eigen::Vector3i> gridDimensions(
    const Eigen::Vector3d &minimum,
    const Eigen::Vector3d &maximum,
    double voxelSize,
    int padding)
{
    Eigen::Vector3i result;
    for (int axis = 0; axis < 3; ++axis) {
        const double cells = std::ceil((maximum[axis] - minimum[axis]) / voxelSize);
        const double dimension = cells + 2.0 * static_cast<double>(padding) + 1.0;
        if (!std::isfinite(dimension) || dimension < 2.0 ||
            dimension > static_cast<double>(std::numeric_limits<int>::max())) {
            return std::nullopt;
        }
        result[axis] = static_cast<int>(dimension);
    }
    return result;
}
}

ReconstructionMethod VoxelReconstructor::method() const
{
    return ReconstructionMethod::Voxel;
}

ReconstructionRequirements VoxelReconstructor::requirements() const
{
    return {NormalRequirement::NotUsed, InputRepresentation::PointCloud};
}

ReconstructionResult VoxelReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing voxel reconstruction");
    const auto *voxelParameters = std::get_if<VoxelReconstructionParameters>(&parameters);
    if (!voxelParameters || !input.pointCloud) {
        result.errorMessage = !voxelParameters
            ? "Voxel reconstruction received parameters for another method."
            : "Voxel reconstruction requires a point cloud.";
        return result;
    }
    if (input.pointCloud->empty()) {
        result.errorMessage = "Voxel reconstruction requires at least one point.";
        return result;
    }
    if (!std::isfinite(voxelParameters->voxelSize) || voxelParameters->voxelSize <= 0.0 ||
        voxelParameters->paddingVoxels < 1 || !std::isfinite(voxelParameters->isoValue) ||
        voxelParameters->isoValue <= 0.0 || voxelParameters->isoValue >= 1.0) {
        result.errorMessage =
            "Voxel size must be positive, padding must be at least one, and iso-value must be between zero and one.";
        return result;
    }

    const auto bounds = input.pointCloud->boundingBox();
    if (!bounds) {
        result.errorMessage = "Point-cloud bounds are unavailable.";
        return result;
    }
    const Eigen::Vector3d minimum = bounds->minPoint().vector();
    const Eigen::Vector3d maximum = bounds->maxPoint().vector();
    const std::optional<Eigen::Vector3i> dimensions = gridDimensions(
        minimum, maximum, voxelParameters->voxelSize, voxelParameters->paddingVoxels);
    if (!dimensions) {
        result.errorMessage = "Voxel-grid dimensions exceed the supported integer range.";
        return result;
    }
    const Eigen::Vector3d origin = minimum -
        static_cast<double>(voxelParameters->paddingVoxels) *
            voxelParameters->voxelSize * Eigen::Vector3d::Ones();
    ScalarGrid3D::CreateResult created = ScalarGrid3D::create(
        *dimensions,
        origin,
        voxelParameters->voxelSize,
        voxelParameters->maximumVoxelCount,
        0.0F);
    if (!created.grid) {
        result.errorMessage = std::move(created.errorMessage);
        return result;
    }
    ScalarGrid3D grid = std::move(*created.grid);
    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.08, "Allocated bounded occupancy grid");

    const auto &points = input.pointCloud->points();
    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        if ((pointIndex & 1023U) == 0U && cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        const Eigen::Array3d continuous =
            (points[pointIndex].vector() - grid.origin()).array() / grid.spacing();
        const Eigen::Vector3i coordinate = continuous.round().cast<int>();
        const std::size_t offsetCount = voxelParameters->closeSmallGaps
            ? kGapClosingOffsets.size()
            : 1U;
        for (std::size_t offsetIndex = 0; offsetIndex < offsetCount; ++offsetIndex) {
            const Eigen::Vector3i target = coordinate + kGapClosingOffsets[offsetIndex];
            if (grid.contains(target.x(), target.y(), target.z())) {
                grid.value(target.x(), target.y(), target.z()) = 1.0F;
            }
        }
        if ((pointIndex & 255U) == 0U || pointIndex + 1U == points.size()) {
            reporter.report(
                ProcessingStage::PropagatingSurface,
                mapPhaseProgress(
                    0.08,
                    0.42,
                    static_cast<double>(pointIndex + 1U) / static_cast<double>(points.size())),
                "Rasterising point samples");
        }
    }

    const ProgressCallback extractionProgress = [&reporter](const ProcessingProgress &progress) {
        reporter.report(
            progress.stage,
            mapPhaseProgress(0.42, 0.98, progress.fraction),
            progress.message);
    };
    MarchingCubesParameters marchingParameters;
    marchingParameters.isoValue = voxelParameters->isoValue;
    result = MarchingCubesReconstructor().extract(
        grid,
        marchingParameters,
        extractionProgress,
        cancellationToken);
    return result;
}
