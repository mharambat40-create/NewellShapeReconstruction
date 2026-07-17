#include "model/pipeline/ReconstructionPipeline.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/normals/NormalMethodRegistry.h"
#include "model/processing/reconstruction/SurfaceReconstructorRegistry.h"
#include "model/processing/spatial/KDTree.h"

#include <chrono>
#include <memory>
#include <utility>

namespace
{
using Clock = std::chrono::steady_clock;

constexpr double kPreparationEnd = ReconstructionProgressWeights::preparation;
constexpr double kSpatialIndexEnd = kPreparationEnd + ReconstructionProgressWeights::spatialIndex;
constexpr double kNormalEstimationEnd = kSpatialIndexEnd + ReconstructionProgressWeights::normalEstimation;
constexpr double kNormalFinalizationEnd = kNormalEstimationEnd + ReconstructionProgressWeights::normalFinalization;
constexpr double kBallPivotingEnd = kNormalFinalizationEnd + ReconstructionProgressWeights::ballPivoting;
constexpr double kResultConstructionEnd = kBallPivotingEnd + ReconstructionProgressWeights::resultConstruction;

double elapsedMilliseconds(const Clock::time_point &start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
}

ReconstructionPipelineResult ReconstructionPipeline::execute(
    const PointCloud &pointCloud,
    const ReconstructionPipelineParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionPipelineResult result;
    ProgressReporter reporter(progressCallback);
    const auto preparationStart = Clock::now();
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing surface reconstruction");
    if (pointCloud.empty()) {
        result.errorMessage = "No point cloud is available for reconstruction.";
        result.timings.preparationMilliseconds = elapsedMilliseconds(preparationStart);
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "Surface reconstruction requires finite point coordinates.";
        result.timings.preparationMilliseconds = elapsedMilliseconds(preparationStart);
        reporter.failed(result.errorMessage);
        return result;
    }

    result.timings.preparationMilliseconds = elapsedMilliseconds(preparationStart);
    const SurfaceReconstructorRegistry registry;
    const ReconstructionAvailability availability = registry.availability(parameters.method);
    if (!availability.available) {
        result.errorMessage = availability.reason;
        reporter.failed(result.errorMessage);
        return result;
    }
    std::unique_ptr<ISurfaceReconstructor> reconstructor = registry.create(parameters.method);
    if (!reconstructor) {
        result.errorMessage = "The selected reconstruction backend could not be created.";
        reporter.failed(result.errorMessage);
        return result;
    }
    const ReconstructionRequirements requirements = reconstructor->requirements();
    if (requirements.input == InputRepresentation::ScalarField ||
        requirements.input == InputRepresentation::VoxelGrid) {
        result.errorMessage =
            "The selected backend requires a scalar or voxel field and cannot consume a point cloud directly.";
        reporter.failed(result.errorMessage);
        return result;
    }

    reporter.report(ProcessingStage::BuildingSpatialIndex, kPreparationEnd, "Building spatial index");
    const auto spatialIndexStart = Clock::now();
    const bool needsNormals = requirements.normals != NormalRequirement::NotUsed;
    const bool needsNeighbourSearch = needsNormals || parameters.method == ReconstructionMethod::GreedyProjection ||
        parameters.method == ReconstructionMethod::BallPivoting;
    std::unique_ptr<KDTree> neighbourSearch;
    if (needsNeighbourSearch) {
        neighbourSearch = std::make_unique<KDTree>(pointCloud);
    }
    result.timings.spatialIndexMilliseconds = elapsedMilliseconds(spatialIndexStart);
    if (cancellationToken.isCancellationRequested()) {
        result.cancelled = true;
        result.errorMessage = "Processing cancelled.";
        reporter.cancelled(result.errorMessage);
        return result;
    }

    double reconstructionStart = kSpatialIndexEnd;
    if (needsNormals) {
        const NormalMethodRegistry normalRegistry;
        const NormalMethod estimationMethod = parameters.normalProcessing.estimationMethod;
        const NormalMethodRequirements estimationRequirements =
            normalRegistry.requirements(estimationMethod);
        if (estimationRequirements.category != NormalMethodCategory::Estimation) {
            result.errorMessage = "The configured first normal stage is not a normal estimator.";
            reporter.failed(result.errorMessage);
            return result;
        }
        std::unique_ptr<INormalEstimator> normalEstimator =
            normalRegistry.createEstimator(estimationMethod);
        if (!normalEstimator) {
            result.errorMessage = "The configured normal estimator could not be created.";
            reporter.failed(result.errorMessage);
            return result;
        }
        const ProgressCallback normalProgress = [&reporter](const ProcessingProgress &progress) {
            reporter.report(
                progress.stage,
                mapPhaseProgress(kSpatialIndexEnd, kNormalEstimationEnd, progress.fraction),
                progress.message);
        };
        const auto normalEstimationStart = Clock::now();
        const NormalMethodParameters estimationParameters =
            estimationMethod == NormalMethod::PcaKNearest
            ? NormalMethodParameters(parameters.normalEstimation)
            : parameters.normalProcessing.estimationParameters;
        NormalEstimationResult normalResult = normalEstimator->estimate(
            pointCloud,
            *neighbourSearch,
            estimationParameters,
            normalProgress,
            cancellationToken);
        result.timings.normalEstimationMilliseconds = elapsedMilliseconds(normalEstimationStart);
        if (!normalResult.succeeded) {
            result.cancelled = normalResult.cancelled;
            result.errorMessage = std::move(normalResult.errorMessage);
            if (result.cancelled) {
                reporter.cancelled(result.errorMessage);
            } else {
                reporter.failed(result.errorMessage);
            }
            return result;
        }
        result.invalidNormalCount = normalResult.invalidPointIndices.size();
        result.normalDiagnostics = normalResult.diagnostics;
        result.normalField = std::move(normalResult.normalField);

        const bool orientationRequired =
            requirements.normals == NormalRequirement::RequiredAndOriented;
        const std::optional<NormalMethod> orientationMethod =
            parameters.normalProcessing.orientationMethod;
        if (orientationMethod) {
            const NormalMethodRequirements orientationRequirements =
                normalRegistry.requirements(*orientationMethod);
            if (orientationRequirements.category != NormalMethodCategory::Orientation) {
                result.errorMessage = "The configured second normal stage is not an orientation method.";
                reporter.failed(result.errorMessage);
                return result;
            }
            std::unique_ptr<INormalOrienter> orienter =
                normalRegistry.createOrienter(*orientationMethod);
            if (!orienter) {
                result.errorMessage = "The configured normal orienter could not be created.";
                reporter.failed(result.errorMessage);
                return result;
            }
            const ProgressCallback orientationProgress = [&reporter](const ProcessingProgress &progress) {
                reporter.report(
                    progress.stage,
                    mapPhaseProgress(
                        kNormalEstimationEnd,
                        kNormalFinalizationEnd,
                        progress.fraction),
                    progress.message);
            };
            const auto orientationStart = Clock::now();
            NormalOrientationResult orientationResult = orienter->orient(
                pointCloud,
                result.normalField,
                *neighbourSearch,
                parameters.normalProcessing.orientationParameters,
                orientationProgress,
                cancellationToken);
            result.timings.normalOrientationMilliseconds =
                elapsedMilliseconds(orientationStart);
            if (!orientationResult.succeeded) {
                result.cancelled = orientationResult.cancelled;
                result.errorMessage = std::move(orientationResult.errorMessage);
                if (result.cancelled) {
                    reporter.cancelled(result.errorMessage);
                } else {
                    reporter.failed(result.errorMessage);
                }
                return result;
            }
            result.normalField = std::move(orientationResult.normalField);
            result.normalDiagnostics = orientationResult.diagnostics;
        }
        if (orientationRequired && !result.normalField.consistentlyOriented()) {
            result.errorMessage =
                "The selected reconstruction method requires consistently oriented normals, "
                "but the current point-cloud workflow provides estimation only.";
            reporter.failed(result.errorMessage);
            return result;
        }
        reporter.report(
            ProcessingStage::Finalizing,
            kNormalFinalizationEnd,
            result.normalField.consistentlyOriented()
                ? "Finalizing oriented normals"
                : "Finalizing estimated normals");
        reconstructionStart = kNormalFinalizationEnd;
    }

    const ProgressCallback reconstructionProgress = [&reporter, reconstructionStart](const ProcessingProgress &progress) {
        reporter.report(
            progress.stage,
            mapPhaseProgress(reconstructionStart, kBallPivotingEnd, progress.fraction),
            progress.message);
    };
    const ReconstructionMethodParameters methodParameters =
        parameters.method == ReconstructionMethod::BallPivoting
        ? ReconstructionMethodParameters(parameters.reconstruction)
        : parameters.methodParameters;
    const ReconstructionInput input{
        .pointCloud = &pointCloud,
        .normalField = needsNormals ? &result.normalField : nullptr,
        .neighbourSearch = neighbourSearch.get(),
    };
    ReconstructionResult reconstructionResult = reconstructor->reconstruct(
        input,
        methodParameters,
        reconstructionProgress,
        cancellationToken);
    result.timings.seedSearchMilliseconds = reconstructionResult.timings.seedSearchMilliseconds;
    result.timings.frontPropagationMilliseconds = reconstructionResult.timings.frontPropagationMilliseconds;
    result.timings.meshValidationMilliseconds = reconstructionResult.timings.meshValidationMilliseconds;
    if (!reconstructionResult.succeeded) {
        result.cancelled = reconstructionResult.cancelled;
        result.errorMessage = std::move(reconstructionResult.errorMessage);
        if (result.cancelled) {
            reporter.cancelled(result.errorMessage);
        } else {
            reporter.failed(result.errorMessage);
        }
        return result;
    }

    const auto resultConstructionStart = Clock::now();
    result.mesh = std::move(reconstructionResult.mesh);
    result.diagnostics = reconstructionResult.diagnostics;
    result.timings.resultConstructionMilliseconds = elapsedMilliseconds(resultConstructionStart);
    result.succeeded = true;
    reporter.report(ProcessingStage::Finalizing, kResultConstructionEnd, "Preparing temporary surface result");
    reporter.report(ProcessingStage::Finalizing, 0.99, "Surface result ready for publication");
    return result;
}
