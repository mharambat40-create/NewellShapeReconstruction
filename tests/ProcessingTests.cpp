#include "model/geometry/NormalField.h"
#include "model/geometry/PointCloud.h"
#include "model/pipeline/PreprocessingPipeline.h"
#include "model/pipeline/ReconstructionPipeline.h"
#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/PcaKnnNormalEstimator.h"
#include "model/processing/reconstruction/BallPivotingReconstructor.h"
#include "model/processing/spatial/KDTree.h"

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireMonotonic(const std::vector<ProcessingProgress> &progress)
{
    double previous = 0.0;
    for (const ProcessingProgress &update : progress) {
        require(update.fraction >= previous, "Progress must not decrease.");
        require(update.fraction >= 0.0 && update.fraction <= 1.0, "Progress must remain clamped.");
        previous = update.fraction;
    }
}

PointCloud makePlane(std::size_t sideLength, double spacing)
{
    PointCloud cloud;
    for (std::size_t row = 0; row < sideLength; ++row) {
        for (std::size_t column = 0; column < sideLength; ++column) {
            cloud.addPoint(
                static_cast<double>(column) * spacing,
                static_cast<double>(row) * spacing,
                0.0);
        }
    }
    return cloud;
}

void testPhaseMappingAndReporterGuards()
{
    require(std::abs(mapPhaseProgress(0.2, 0.6, 0.5) - 0.4) < 1.0e-12, "Phase mapping should be linear.");
    require(mapPhaseProgress(0.2, 0.6, -4.0) == 0.2, "Local progress should clamp low.");
    require(mapPhaseProgress(0.2, 0.6, 4.0) == 0.6, "Local progress should clamp high.");
    require(
        mapPhaseProgress(0.2, 0.6, std::numeric_limits<double>::quiet_NaN()) == 0.2,
        "Non-finite local progress should map to the phase start.");

    std::vector<ProcessingProgress> updates;
    ProgressReporter reporter([&updates](const ProcessingProgress &progress) {
        updates.push_back(progress);
    });
    reporter.report(ProcessingStage::Preparing, 0.4);
    reporter.report(ProcessingStage::Preparing, 0.1);
    reporter.report(ProcessingStage::Preparing, 2.0);
    requireMonotonic(updates);
    require(updates.back().fraction < 1.0, "Ordinary reports must not emit 100 percent.");
    reporter.complete();
    require(updates.back().fraction == 1.0, "Only completion should emit 100 percent.");
}

void testCancellationToken()
{
    CancellationSource source;
    const CancellationToken token = source.token();
    require(!token.isCancellationRequested(), "A new token should not be cancelled.");
    source.requestCancellation();
    require(token.isCancellationRequested(), "Cancellation should propagate to copied tokens.");
}

void testPreprocessingProgressAndCancellation()
{
    PointCloud cloud;
    for (int index = 0; index < 300; ++index) {
        cloud.addPoint(static_cast<double>(index % 10), 0.0, 0.0);
    }

    PreprocessingPipelineParameters parameters;
    parameters.operation = PreprocessingOperation::SelectPerfectDuplicates;
    std::vector<ProcessingProgress> updates;
    const PreprocessingPipelineResult result = PreprocessingPipeline().execute(
        cloud,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(result.succeeded, "Perfect-duplicate processing should succeed.");
    require(result.selectedIndices.size() == 290U, "Perfect duplicates should retain one representative.");
    requireMonotonic(updates);
    require(updates.back().fraction == 1.0, "Successful preprocessing should finish at 100 percent.");

    CancellationSource source;
    source.requestCancellation();
    const PreprocessingPipelineResult cancelled = PreprocessingPipeline().execute(
        cloud,
        parameters,
        {},
        source.token());
    require(cancelled.cancelled && !cancelled.succeeded, "Preprocessing should honour cancellation.");

    updates.clear();
    const PreprocessingPipelineResult emptyResult = PreprocessingPipeline().execute(
        PointCloud{},
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(!emptyResult.succeeded, "Empty preprocessing input should fail safely.");
    for (const ProcessingProgress &update : updates) {
        require(update.fraction < 1.0, "Empty preprocessing input must not report completion.");
    }
}

void testPcaProgressAndCancellation()
{
    const PointCloud cloud = makePlane(20U, 0.1);
    const KDTree tree(cloud);
    NormalEstimationParameters parameters;
    parameters.neighbourCount = 12U;
    std::vector<ProcessingProgress> updates;
    const NormalEstimationResult result = PcaKnnNormalEstimator().estimate(
        cloud,
        tree,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(result.succeeded, "PCA should succeed on a regular plane.");
    requireMonotonic(updates);
    require(updates.back().fraction <= 0.99, "Worker-side PCA progress must not report completion.");

    CancellationSource source;
    source.requestCancellation();
    const NormalEstimationResult cancelled = PcaKnnNormalEstimator().estimate(
        cloud,
        tree,
        parameters,
        {},
        source.token());
    require(cancelled.cancelled && !cancelled.succeeded, "PCA should honour cancellation.");
}

void testBallPivotingProgressAndFailure()
{
    const PointCloud cloud = makePlane(6U, 0.5);
    NormalField normals;
    for (std::size_t index = 0; index < cloud.pointCount(); ++index) {
        normals.addNormal(Eigen::Vector3d::UnitZ());
    }
    const KDTree tree(cloud);
    ReconstructionParameters parameters;
    parameters.ballRadius = 0.45;
    std::vector<ProcessingProgress> updates;
    const ReconstructionResult result = BallPivotingReconstructor().reconstruct(
        cloud,
        normals,
        tree,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(result.succeeded, "Ball Pivoting should succeed on the test plane.");
    requireMonotonic(updates);
    require(updates.back().fraction <= 0.99, "Worker-side BPA progress must not report completion.");

    PointCloud emptyCloud;
    NormalField emptyNormals;
    const KDTree emptyTree(emptyCloud);
    updates.clear();
    const ReconstructionResult failed = BallPivotingReconstructor().reconstruct(
        emptyCloud,
        emptyNormals,
        emptyTree,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(!failed.succeeded, "Empty input should fail reconstruction.");
    for (const ProcessingProgress &update : updates) {
        require(update.fraction < 1.0, "Failed reconstruction must not emit 100 percent.");
    }
}

void testSurfacePipelineProgressDistribution()
{
    const double weightTotal =
        ReconstructionProgressWeights::preparation +
        ReconstructionProgressWeights::spatialIndex +
        ReconstructionProgressWeights::normalEstimation +
        ReconstructionProgressWeights::normalFinalization +
        ReconstructionProgressWeights::ballPivoting +
        ReconstructionProgressWeights::resultConstruction +
        ReconstructionProgressWeights::publication;
    require(std::abs(weightTotal - 1.0) < 1.0e-12, "Surface progress weights should sum to one.");
    require(
        ReconstructionProgressWeights::ballPivoting > 0.5,
        "Ball Pivoting should receive the dominant global progress range.");

    const double propagationStart = mapPhaseProgress(0.30, 0.98, 0.12);
    const double propagationEnd = mapPhaseProgress(0.30, 0.98, 0.94);
    require(propagationStart > 0.37 && propagationStart < 0.39, "Propagation should begin after seed search.");
    require(propagationEnd > 0.93 && propagationEnd < 0.95, "Propagation should occupy the broad central range.");

    const PointCloud cloud = makePlane(8U, 0.4);
    ReconstructionPipelineParameters parameters;
    parameters.normalEstimation.neighbourCount = 12U;
    parameters.normalEstimation.maximumSearchRadius = 1.0;
    parameters.reconstruction.ballRadius = 0.38;
    std::vector<ProcessingProgress> updates;
    const ReconstructionPipelineResult result = ReconstructionPipeline().execute(
        cloud,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(result.succeeded, "The complete reconstruction pipeline should succeed on a regular plane.");
    requireMonotonic(updates);
    for (const ProcessingProgress &update : updates) {
        require(update.fraction <= 0.99, "Worker pipeline progress must remain below 100 percent.");
    }
    require(result.timings.frontPropagationMilliseconds >= 0.0, "BPA timing data should be available.");

    updates.clear();
    const ReconstructionPipelineResult failed = ReconstructionPipeline().execute(
        PointCloud{},
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); });
    require(!failed.succeeded, "Empty reconstruction input should fail safely.");
    for (const ProcessingProgress &update : updates) {
        require(update.fraction < 1.0, "Failed conversion must not report completion.");
    }

    CancellationSource cancellationSource;
    cancellationSource.requestCancellation();
    updates.clear();
    const ReconstructionPipelineResult cancelled = ReconstructionPipeline().execute(
        cloud,
        parameters,
        [&updates](const ProcessingProgress &progress) { updates.push_back(progress); },
        cancellationSource.token());
    require(cancelled.cancelled && !cancelled.succeeded, "Surface conversion should honour cancellation.");
    for (const ProcessingProgress &update : updates) {
        require(update.fraction < 1.0, "Cancelled conversion must not report completion.");
    }
}
}

int main()
{
    try {
        testPhaseMappingAndReporterGuards();
        testCancellationToken();
        testPreprocessingProgressAndCancellation();
        testPcaProgressAndCancellation();
        testBallPivotingProgressAndFailure();
        testSurfacePipelineProgressDistribution();
    } catch (const std::exception &exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    return 0;
}
