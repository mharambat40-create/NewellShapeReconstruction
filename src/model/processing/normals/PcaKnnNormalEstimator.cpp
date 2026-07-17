#include "model/processing/normals/PcaKnnNormalEstimator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <algorithm>
#include <cmath>

NormalMethod PcaKnnNormalEstimator::method() const
{
    return NormalMethod::PcaKNearest;
}

NormalEstimationResult PcaKnnNormalEstimator::estimate(
    const PointCloud &pointCloud,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<PcaKnnParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "PCA k-nearest received parameters for another normal method.";
        return result;
    }
    if (pointCloud.pointCount() < 4U) {
        result.errorMessage = "At least four points are required for PCA normal estimation.";
        return result;
    }
    if (methodParameters->neighbourCount < 3U) {
        result.errorMessage = "PCA normal estimation requires at least three neighbours.";
        return result;
    }
    if (methodParameters->maximumSearchRadius <= 0.0 ||
        methodParameters->degeneracyTolerance <= 0.0) {
        result.errorMessage = "Normal-estimation tolerances must be positive.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "PCA normal estimation requires finite point coordinates.";
        return result;
    }

    result.normalField.reserve(pointCloud.pointCount());
    result.invalidPointIndices.reserve(pointCloud.pointCount() / 20U);
    const double maximumSquaredRadius =
        methodParameters->maximumSearchRadius * methodParameters->maximumSearchRadius;
    reporter.report(ProcessingStage::EstimatingNormals, 0.0, "Estimating point normals");

    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        if (pointIndex % 128U == 0U) {
            reporter.report(
                ProcessingStage::EstimatingNormals,
                static_cast<double>(pointIndex) / static_cast<double>(pointCloud.pointCount()),
                "Estimating point normals");
        }

        std::vector<Neighbour> neighbours = neighbourSearch.kNearest(
            pointCloud.points()[pointIndex].vector(),
            methodParameters->neighbourCount,
            pointIndex);
        neighbours.erase(
            std::remove_if(
                neighbours.begin(),
                neighbours.end(),
                [maximumSquaredRadius](const Neighbour &neighbour) {
                    return neighbour.squaredDistance > maximumSquaredRadius;
                }),
            neighbours.end());

        if (neighbours.size() < 3U) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            continue;
        }

        const LocalPlaneEstimate estimate = estimateLocalPlane(
            pointCloud,
            neighbours,
            methodParameters->degeneracyTolerance);
        if (!estimate.valid) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.rejectedNeighbourhoodCount;
            continue;
        }

        result.normalField.addNormal(estimate.normal, estimate.confidence);
    }

    reporter.report(ProcessingStage::EstimatingNormals, 0.99, "Estimating point normals");

    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "No stable PCA normals could be estimated from the point cloud.";
        return result;
    }

    finalizeNormalDiagnostics(
        result.normalField,
        pointCloud.pointCount(),
        result.diagnostics);
    result.succeeded = true;
    reporter.report(ProcessingStage::EstimatingNormals, 0.99, "Normal estimation complete");
    return result;
}
