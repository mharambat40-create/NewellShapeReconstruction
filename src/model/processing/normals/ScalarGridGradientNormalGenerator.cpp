#include "model/processing/normals/ScalarGridGradientNormalGenerator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"

#include <algorithm>
#include <cmath>

namespace
{
std::optional<Eigen::Vector3d> gradientAtGridPoint(
    const ScalarGrid3D &grid,
    int x,
    int y,
    int z)
{
    Eigen::Vector3d gradient;
    const Eigen::Vector3i coordinate(x, y, z);
    for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3i negative = coordinate;
        Eigen::Vector3i positive = coordinate;
        if (coordinate[axis] == 0) {
            positive[axis] += 1;
            const double current = grid.value(x, y, z);
            const double next = grid.value(positive.x(), positive.y(), positive.z());
            gradient[axis] = (next - current) / grid.spacing();
        } else if (coordinate[axis] + 1 == grid.dimensions()[axis]) {
            negative[axis] -= 1;
            const double previous = grid.value(negative.x(), negative.y(), negative.z());
            const double current = grid.value(x, y, z);
            gradient[axis] = (current - previous) / grid.spacing();
        } else {
            negative[axis] -= 1;
            positive[axis] += 1;
            const double previous = grid.value(negative.x(), negative.y(), negative.z());
            const double next = grid.value(positive.x(), positive.y(), positive.z());
            gradient[axis] = (next - previous) / (2.0 * grid.spacing());
        }
    }
    if (!gradient.allFinite()) {
        return std::nullopt;
    }
    return gradient;
}

std::optional<Eigen::Vector3d> gradientAtPosition(
    const ScalarGrid3D &grid,
    const Eigen::Vector3d &position,
    bool interpolate)
{
    if (!position.allFinite()) {
        return std::nullopt;
    }
    const Eigen::Vector3d gridCoordinate = (position - grid.origin()) / grid.spacing();
    for (int axis = 0; axis < 3; ++axis) {
        if (gridCoordinate[axis] < 0.0 ||
            gridCoordinate[axis] > static_cast<double>(grid.dimensions()[axis] - 1)) {
            return std::nullopt;
        }
    }
    if (!interpolate) {
        const Eigen::Vector3i nearest(
            static_cast<int>(std::lround(gridCoordinate.x())),
            static_cast<int>(std::lround(gridCoordinate.y())),
            static_cast<int>(std::lround(gridCoordinate.z())));
        return gradientAtGridPoint(grid, nearest.x(), nearest.y(), nearest.z());
    }

    const Eigen::Vector3i lower(
        static_cast<int>(std::floor(gridCoordinate.x())),
        static_cast<int>(std::floor(gridCoordinate.y())),
        static_cast<int>(std::floor(gridCoordinate.z())));
    const Eigen::Vector3i upper(
        std::min(lower.x() + 1, grid.dimensions().x() - 1),
        std::min(lower.y() + 1, grid.dimensions().y() - 1),
        std::min(lower.z() + 1, grid.dimensions().z() - 1));
    const Eigen::Vector3d fraction = gridCoordinate - lower.cast<double>();
    Eigen::Vector3d interpolated = Eigen::Vector3d::Zero();
    for (int dz = 0; dz <= 1; ++dz) {
        for (int dy = 0; dy <= 1; ++dy) {
            for (int dx = 0; dx <= 1; ++dx) {
                const int sampleX = dx == 0 ? lower.x() : upper.x();
                const int sampleY = dy == 0 ? lower.y() : upper.y();
                const int sampleZ = dz == 0 ? lower.z() : upper.z();
                const std::optional<Eigen::Vector3d> sample =
                    gradientAtGridPoint(grid, sampleX, sampleY, sampleZ);
                if (!sample) {
                    return std::nullopt;
                }
                const double weight =
                    (dx == 0 ? 1.0 - fraction.x() : fraction.x()) *
                    (dy == 0 ? 1.0 - fraction.y() : fraction.y()) *
                    (dz == 0 ? 1.0 - fraction.z() : fraction.z());
                interpolated += weight * *sample;
            }
        }
    }
    return interpolated;
}
}

NormalMethod ScalarGridGradientNormalGenerator::method() const
{
    return NormalMethod::ScalarGridGradient;
}

NormalEstimationResult ScalarGridGradientNormalGenerator::generate(
    const FieldNormalInput &input,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<ScalarGridGradientParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Scalar-grid gradients received parameters for another normal method.";
        return result;
    }
    if (!input.scalarGrid) {
        result.errorMessage = "Scalar-grid gradients require a voxel or scalar grid.";
        return result;
    }
    if (!std::isfinite(methodParameters->minimumGradientMagnitude) ||
        methodParameters->minimumGradientMagnitude <= 0.0) {
        result.errorMessage = "The minimum scalar-grid gradient magnitude must be positive and finite.";
        return result;
    }

    const std::size_t queryCount =
        input.queryPoints ? input.queryPoints->pointCount() : input.scalarGrid->voxelCount();
    if (queryCount == 0U) {
        result.errorMessage = "Scalar-grid gradients require query positions or grid samples.";
        return result;
    }
    result.normalField.reserve(queryCount);
    result.invalidPointIndices.reserve(queryCount / 20U);
    reporter.report(ProcessingStage::EvaluatingFieldGradients, 0.0, "Evaluating scalar-grid gradients");

    for (std::size_t index = 0; index < queryCount; ++index) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }

        std::optional<Eigen::Vector3d> gradient;
        if (input.queryPoints) {
            gradient = gradientAtPosition(
                *input.scalarGrid,
                input.queryPoints->points()[index].vector(),
                methodParameters->interpolateGradient);
        } else {
            const std::size_t width = static_cast<std::size_t>(input.scalarGrid->dimensions().x());
            const std::size_t height = static_cast<std::size_t>(input.scalarGrid->dimensions().y());
            const int x = static_cast<int>(index % width);
            const int y = static_cast<int>((index / width) % height);
            const int z = static_cast<int>(index / (width * height));
            gradient = gradientAtGridPoint(*input.scalarGrid, x, y, z);
        }

        const double magnitude = gradient ? gradient->norm() : 0.0;
        if (!gradient || !gradient->allFinite() || !std::isfinite(magnitude) ||
            magnitude < methodParameters->minimumGradientMagnitude) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(index);
            ++result.diagnostics.nearZeroGradientCount;
        } else {
            Eigen::Vector3d direction = *gradient / magnitude;
            if (methodParameters->invertDirection) {
                direction = -direction;
            }
            result.normalField.addNormal(
                direction,
                std::clamp(
                    magnitude / (magnitude + methodParameters->minimumGradientMagnitude),
                    0.0,
                    1.0));
        }
        if ((index & 255U) == 0U) {
            reporter.report(
                ProcessingStage::EvaluatingFieldGradients,
                static_cast<double>(index) / static_cast<double>(queryCount),
                "Evaluating scalar-grid gradients");
        }
    }

    result.normalField.setConsistentlyOriented(true);
    finalizeNormalDiagnostics(result.normalField, queryCount, result.diagnostics);
    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "The scalar grid produced no usable gradients.";
        return result;
    }
    result.succeeded = true;
    reporter.report(ProcessingStage::EvaluatingFieldGradients, 0.99, "Scalar-grid normals ready");
    return result;
}
