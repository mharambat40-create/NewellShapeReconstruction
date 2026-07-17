#include "model/processing/normals/MultiScalePcaNormalEstimator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
struct ScaleCandidate
{
    LocalPlaneEstimate estimate;
    std::size_t neighbourCount = 0U;
    double score = 0.0;
};
}

NormalMethod MultiScalePcaNormalEstimator::method() const
{
    return NormalMethod::PcaMultiScale;
}

NormalEstimationResult MultiScalePcaNormalEstimator::estimate(
    const PointCloud &pointCloud,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<MultiScalePcaParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Multi-scale PCA received parameters for another normal method.";
        return result;
    }
    if (pointCloud.pointCount() < 4U || methodParameters->neighbourCounts.empty() ||
        methodParameters->minimumValidScales == 0U ||
        !std::isfinite(methodParameters->curvatureWeight) ||
        !std::isfinite(methodParameters->stabilityWeight) ||
        methodParameters->curvatureWeight < 0.0 || methodParameters->stabilityWeight < 0.0 ||
        methodParameters->curvatureWeight + methodParameters->stabilityWeight <= 0.0 ||
        !std::isfinite(methodParameters->degeneracyTolerance) ||
        methodParameters->degeneracyTolerance <= 0.0) {
        result.errorMessage = "Multi-scale PCA parameters are invalid.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "Multi-scale PCA requires finite point coordinates.";
        return result;
    }

    std::vector<std::size_t> scales = methodParameters->neighbourCounts;
    std::sort(scales.begin(), scales.end());
    scales.erase(std::unique(scales.begin(), scales.end()), scales.end());
    scales.erase(
        std::remove_if(scales.begin(), scales.end(), [](std::size_t count) {
            return count < 3U;
        }),
        scales.end());
    if (scales.size() < methodParameters->minimumValidScales) {
        result.errorMessage = "Multi-scale PCA does not contain enough valid neighbourhood sizes.";
        return result;
    }

    result.normalField.reserve(pointCloud.pointCount());
    result.invalidPointIndices.reserve(pointCloud.pointCount() / 10U);
    result.diagnostics.selectedNeighbourCounts.assign(pointCloud.pointCount(), 0U);
    const double totalEvaluations =
        static_cast<double>(pointCloud.pointCount()) * static_cast<double>(scales.size());
    std::size_t completedEvaluations = 0U;
    reporter.report(ProcessingStage::EstimatingNormals, 0.0, "Evaluating multiple PCA scales");

    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        std::vector<ScaleCandidate> candidates;
        candidates.reserve(scales.size());
        for (const std::size_t scale : scales) {
            if (cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            const std::vector<Neighbour> neighbours = neighbourSearch.kNearest(
                pointCloud.points()[pointIndex].vector(),
                scale,
                pointIndex);
            const LocalPlaneEstimate estimate = estimateLocalPlane(
                pointCloud,
                neighbours,
                methodParameters->degeneracyTolerance);
            if (estimate.valid) {
                candidates.push_back(ScaleCandidate{estimate, scale, 0.0});
            }
            ++completedEvaluations;
            if ((completedEvaluations & 127U) == 0U) {
                reporter.report(
                    ProcessingStage::EstimatingNormals,
                    static_cast<double>(completedEvaluations) / totalEvaluations,
                    "Evaluating multiple PCA scales");
            }
        }

        if (candidates.size() < methodParameters->minimumValidScales) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.rejectedNeighbourhoodCount;
            continue;
        }

        for (std::size_t index = 0; index < candidates.size(); ++index) {
            double stability = 0.0;
            std::size_t comparisons = 0U;
            if (index > 0U) {
                stability += std::abs(
                    candidates[index].estimate.normal.dot(candidates[index - 1U].estimate.normal));
                ++comparisons;
            }
            if (index + 1U < candidates.size()) {
                stability += std::abs(
                    candidates[index].estimate.normal.dot(candidates[index + 1U].estimate.normal));
                ++comparisons;
            }
            stability = comparisons > 0U ? stability / static_cast<double>(comparisons) : 1.0;
            candidates[index].score =
                methodParameters->curvatureWeight * (1.0 - candidates[index].estimate.surfaceVariation) +
                methodParameters->stabilityWeight * stability;
        }

        const auto selected = std::max_element(
            candidates.begin(),
            candidates.end(),
            [](const ScaleCandidate &left, const ScaleCandidate &right) {
                if (left.score == right.score) {
                    return left.neighbourCount > right.neighbourCount;
                }
                return left.score < right.score;
            });
        const double weightSum =
            methodParameters->curvatureWeight + methodParameters->stabilityWeight;
        const double confidence = std::clamp(selected->score / weightSum, 0.0, 1.0);
        result.normalField.addNormal(selected->estimate.normal, confidence);
        result.diagnostics.selectedNeighbourCounts[pointIndex] = selected->neighbourCount;
    }

    finalizeNormalDiagnostics(result.normalField, pointCloud.pointCount(), result.diagnostics);
    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "No stable multi-scale PCA normals could be estimated.";
        return result;
    }
    result.succeeded = true;
    reporter.report(ProcessingStage::EstimatingNormals, 0.99, "Multi-scale PCA normals ready");
    return result;
}
