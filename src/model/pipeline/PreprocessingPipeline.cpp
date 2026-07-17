#include "model/pipeline/PreprocessingPipeline.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/preprocessing/SelectionAlgorithms.h"

#include <utility>

PreprocessingPipelineResult PreprocessingPipeline::execute(
    const PointCloud &pointCloud,
    const PreprocessingPipelineParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing point cloud");

    const ProgressCallback algorithmProgress = [&reporter](const ProcessingProgress &progress) {
        reporter.report(
            progress.stage,
            mapPhaseProgress(0.05, 0.95, progress.fraction),
            progress.message);
    };

    PointSelectionResult selectionResult;
    switch (parameters.operation) {
    case PreprocessingOperation::SelectSparsePoints:
        selectionResult = InvalidPointSelector().selectSparsePoints(
            pointCloud,
            parameters.radius,
            parameters.minimumNeighbourCount,
            algorithmProgress,
            cancellationToken);
        break;
    case PreprocessingOperation::SelectPerfectDuplicates:
        selectionResult = DuplicatePointSelector().selectPerfectDuplicates(
            pointCloud,
            algorithmProgress,
            cancellationToken);
        break;
    case PreprocessingOperation::SelectNearDuplicates:
        selectionResult = DuplicatePointSelector().selectNearDuplicates(
            pointCloud,
            parameters.distanceThreshold,
            algorithmProgress,
            cancellationToken);
        break;
    }

    if (selectionResult.cancelled) {
        reporter.cancelled("Processing cancelled.");
        return PreprocessingPipelineResult{false, true, {}, "Processing cancelled."};
    }
    if (!selectionResult.succeeded) {
        reporter.failed(selectionResult.errorMessage);
        return PreprocessingPipelineResult{false, false, {}, std::move(selectionResult.errorMessage)};
    }

    reporter.report(ProcessingStage::Finalizing, 0.99, "Finalizing selection");
    reporter.complete("Selection complete");
    return PreprocessingPipelineResult{
        true,
        false,
        std::move(selectionResult.selectedIndices),
        {},
    };
}
