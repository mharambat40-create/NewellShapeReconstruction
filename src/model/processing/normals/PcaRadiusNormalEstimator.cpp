#include "model/processing/normals/PcaRadiusNormalEstimator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <algorithm>
#include <cmath>

NormalMethod PcaRadiusNormalEstimator::method() const
{
    return NormalMethod::PcaFixedRadius;
}

NormalEstimationResult PcaRadiusNormalEstimator::estimate(
    const PointCloud &pointCloud,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<PcaRadiusParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "PCA fixed radius received parameters for another normal method.";
        return result;
    }
    if (pointCloud.pointCount() < 4U) {
        result.errorMessage = "PCA fixed radius requires at least four points.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "PCA fixed radius requires finite point coordinates.";
        return result;
    }
    if (!std::isfinite(methodParameters->searchRadius) || methodParameters->searchRadius <= 0.0 ||
        methodParameters->minimumNeighbours < 3U ||
        methodParameters->maximumNeighbours < methodParameters->minimumNeighbours ||
        !std::isfinite(methodParameters->degeneracyTolerance) ||
        methodParameters->degeneracyTolerance <= 0.0) {
        result.errorMessage =
            "PCA fixed-radius parameters require a positive radius, at least three neighbours, and a valid maximum.";
        return result;
    }

    result.normalField.reserve(pointCloud.pointCount());
    result.invalidPointIndices.reserve(pointCloud.pointCount() / 10U);
    reporter.report(ProcessingStage::EstimatingNormals, 0.0, "Estimating fixed-radius local planes");
    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        if ((pointIndex & 127U) == 0U) {
            reporter.report(
                ProcessingStage::EstimatingNormals,
                static_cast<double>(pointIndex) / static_cast<double>(pointCloud.pointCount()),
                "Estimating fixed-radius local planes");
        }

        std::vector<Neighbour> neighbours = neighbourSearch.radiusSearch(
            pointCloud.points()[pointIndex].vector(),
            methodParameters->searchRadius,
            pointIndex);
        if (neighbours.size() > methodParameters->maximumNeighbours) {
            neighbours.resize(methodParameters->maximumNeighbours);
        }
        if (neighbours.size() < methodParameters->minimumNeighbours) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.rejectedNeighbourhoodCount;
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

    finalizeNormalDiagnostics(result.normalField, pointCloud.pointCount(), result.diagnostics);
    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "No stable fixed-radius PCA normals could be estimated.";
        return result;
    }
    result.succeeded = true;
    reporter.report(ProcessingStage::EstimatingNormals, 0.99, "Fixed-radius normals ready");
    return result;
}
