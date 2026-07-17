#include "model/processing/reconstruction/RbfReconstructor.h"

#include "model/geometry/NormalField.h"
#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/reconstruction/MarchingCubesReconstructor.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
constexpr std::size_t kHardMaximumControlPoints = 512U;

double kernelValue(double distance, const RbfParameters &parameters)
{
    const double scaled = distance / parameters.supportRadius;
    switch (parameters.kernel) {
    case RbfKernel::WendlandC2:
        if (scaled >= 1.0) {
            return 0.0;
        }
        return std::pow(1.0 - scaled, 4.0) * (4.0 * scaled + 1.0);
    case RbfKernel::Gaussian:
        return std::exp(-scaled * scaled);
    case RbfKernel::Multiquadric:
        return std::sqrt(1.0 + scaled * scaled);
    }
    return 0.0;
}

std::optional<Eigen::Vector3i> dimensionsForBounds(
    const Eigen::Vector3d &minimum,
    const Eigen::Vector3d &maximum,
    double spacing)
{
    Eigen::Vector3i dimensions;
    for (int axis = 0; axis < 3; ++axis) {
        const double dimension = std::ceil((maximum[axis] - minimum[axis]) / spacing) + 1.0;
        if (!std::isfinite(dimension) || dimension < 2.0 ||
            dimension > static_cast<double>(std::numeric_limits<int>::max())) {
            return std::nullopt;
        }
        dimensions[axis] = static_cast<int>(dimension);
    }
    return dimensions;
}
}

ReconstructionMethod RbfReconstructor::method() const
{
    return ReconstructionMethod::Rbf;
}

ReconstructionRequirements RbfReconstructor::requirements() const
{
    return {NormalRequirement::RequiredAndOriented, InputRepresentation::OrientedPointCloud};
}

ReconstructionResult RbfReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing RBF reconstruction");
    const auto *methodParameters = std::get_if<RbfParameters>(&parameters);
    if (!methodParameters || !input.pointCloud || !input.normalField) {
        result.errorMessage = "RBF reconstruction requires matching parameters, points and oriented normals.";
        return result;
    }
    if (input.pointCloud->pointCount() < 3U ||
        input.normalField->size() != input.pointCloud->pointCount() ||
        input.normalField->validNormalCount() != input.pointCloud->pointCount()) {
        result.errorMessage = "RBF reconstruction requires one valid oriented normal per point.";
        return result;
    }
    if (!std::isfinite(methodParameters->supportRadius) || methodParameters->supportRadius <= 0.0 ||
        !std::isfinite(methodParameters->regularisation) || methodParameters->regularisation < 0.0 ||
        !std::isfinite(methodParameters->gridSpacing) || methodParameters->gridSpacing <= 0.0 ||
        methodParameters->maximumControlPoints < 3U ||
        methodParameters->maximumControlPoints > kHardMaximumControlPoints ||
        methodParameters->maximumVoxelCount == 0U ||
        methodParameters->maximumFieldEvaluations == 0U) {
        result.errorMessage = "RBF parameters exceed their finite, positive safety bounds.";
        return result;
    }

    const std::size_t controlCount = std::min(
        input.pointCloud->pointCount(), methodParameters->maximumControlPoints);
    const std::size_t constraintCount = controlCount * 3U;
    std::vector<Eigen::Vector3d> constraintPoints;
    constraintPoints.reserve(constraintCount);
    Eigen::VectorXd constraintValues(static_cast<Eigen::Index>(constraintCount));
    const double offsetDistance = std::min(
        0.25 * methodParameters->supportRadius,
        0.5 * methodParameters->gridSpacing);
    for (std::size_t control = 0; control < controlCount; ++control) {
        const std::size_t pointIndex = control * input.pointCloud->pointCount() / controlCount;
        const Eigen::Vector3d &point = input.pointCloud->points()[pointIndex].vector();
        const Eigen::Vector3d &normal = input.normalField->normal(pointIndex).vector();
        constraintPoints.push_back(point);
        constraintValues[static_cast<Eigen::Index>(3U * control)] = 0.0;
        constraintPoints.push_back(point + offsetDistance * normal);
        constraintValues[static_cast<Eigen::Index>(3U * control + 1U)] = offsetDistance;
        constraintPoints.push_back(point - offsetDistance * normal);
        constraintValues[static_cast<Eigen::Index>(3U * control + 2U)] = -offsetDistance;
    }
    reporter.report(ProcessingStage::Preparing, 0.08, "Selected bounded RBF controls");

    Eigen::MatrixXd system(
        static_cast<Eigen::Index>(constraintCount),
        static_cast<Eigen::Index>(constraintCount));
    for (std::size_t row = 0; row < constraintCount; ++row) {
        if ((row & 31U) == 0U && cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        for (std::size_t column = 0; column < constraintCount; ++column) {
            system(
                static_cast<Eigen::Index>(row),
                static_cast<Eigen::Index>(column)) = kernelValue(
                (constraintPoints[row] - constraintPoints[column]).norm(),
                *methodParameters);
        }
        system(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(row)) +=
            methodParameters->regularisation;
        reporter.report(
            ProcessingStage::BuildingSpatialIndex,
            mapPhaseProgress(0.08, 0.30, static_cast<double>(row + 1U) / constraintCount),
            "Assembling bounded RBF system");
    }

    reporter.report(ProcessingStage::PropagatingSurface, 0.31, "Solving RBF coefficients");
    const Eigen::VectorXd weights = system.colPivHouseholderQr().solve(constraintValues);
    if (!weights.allFinite() ||
        (system * weights - constraintValues).norm() >
            0.25 * std::max(1.0, constraintValues.norm())) {
        result.errorMessage = "RBF system is singular or numerically unstable.";
        return result;
    }
    if (cancellationToken.isCancellationRequested()) {
        result.cancelled = true;
        result.errorMessage = "Processing cancelled.";
        reporter.cancelled(result.errorMessage);
        return result;
    }
    reporter.report(ProcessingStage::PropagatingSurface, 0.42, "RBF system solved");

    const auto bounds = input.pointCloud->boundingBox();
    const double padding = std::max(methodParameters->supportRadius, 2.0 * methodParameters->gridSpacing);
    const Eigen::Vector3d minimum = bounds->minPoint().vector() - padding * Eigen::Vector3d::Ones();
    const Eigen::Vector3d maximum = bounds->maxPoint().vector() + padding * Eigen::Vector3d::Ones();
    const std::optional<Eigen::Vector3i> dimensions = dimensionsForBounds(
        minimum, maximum, methodParameters->gridSpacing);
    if (!dimensions) {
        result.errorMessage = "RBF sampling-grid dimensions exceed the supported range.";
        return result;
    }
    ScalarGrid3D::CreateResult created = ScalarGrid3D::create(
        *dimensions,
        minimum,
        methodParameters->gridSpacing,
        methodParameters->maximumVoxelCount);
    if (!created.grid) {
        result.errorMessage = std::move(created.errorMessage);
        return result;
    }
    ScalarGrid3D grid = std::move(*created.grid);
    if (constraintCount > methodParameters->maximumFieldEvaluations / grid.voxelCount()) {
        result.errorMessage = "RBF field sampling exceeds the configured evaluation-count limit.";
        return result;
    }

    for (int z = 0; z < grid.dimensions().z(); ++z) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int x = 0; x < grid.dimensions().x(); ++x) {
                const Eigen::Vector3d position = grid.position(x, y, z);
                double value = 0.0;
                for (std::size_t constraint = 0; constraint < constraintCount; ++constraint) {
                    value += weights[static_cast<Eigen::Index>(constraint)] * kernelValue(
                        (position - constraintPoints[constraint]).norm(), *methodParameters);
                }
                grid.value(x, y, z) = static_cast<float>(value);
            }
        }
        reporter.report(
            ProcessingStage::PropagatingSurface,
            mapPhaseProgress(
                0.42,
                0.80,
                static_cast<double>(z + 1) / static_cast<double>(grid.dimensions().z())),
            "Sampling RBF implicit field");
    }

    const ProgressCallback extractionProgress = [&reporter](const ProcessingProgress &progress) {
        reporter.report(
            progress.stage,
            mapPhaseProgress(0.80, 0.98, progress.fraction),
            progress.message);
    };
    MarchingCubesParameters marchingParameters;
    marchingParameters.isoValue = 0.0;
    return MarchingCubesReconstructor().extract(
        grid,
        marchingParameters,
        extractionProgress,
        cancellationToken);
}
