#include "model/processing/normals/ViewpointNormalOrienter.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"

NormalMethod ViewpointNormalOrienter::method() const
{
    return NormalMethod::ViewpointOrientation;
}

NormalOrientationResult ViewpointNormalOrienter::orient(
    const PointCloud &pointCloud,
    const NormalField &inputNormals,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    (void)neighbourSearch;
    NormalOrientationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<ViewpointOrientationParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Viewpoint orientation received parameters for another normal method.";
        return result;
    }
    if (!methodParameters->viewpoint.allFinite()) {
        result.errorMessage = "Viewpoint orientation requires a finite viewpoint.";
        return result;
    }
    if (inputNormals.size() != pointCloud.pointCount() || inputNormals.validNormalCount() == 0U) {
        result.errorMessage = "Viewpoint orientation requires one normal entry per point and valid normals.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "Viewpoint orientation requires finite point coordinates.";
        return result;
    }

    result.normalField.reserve(inputNormals.size());
    reporter.report(ProcessingStage::OrientingNormals, 0.0, "Orienting normals from viewpoint");
    for (std::size_t index = 0; index < inputNormals.size(); ++index) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        const Normal3d &inputNormal = inputNormals.normal(index);
        if (!inputNormal.isValid()) {
            result.normalField.addInvalidNormal();
            continue;
        }
        Eigen::Vector3d direction = inputNormal.vector();
        const Eigen::Vector3d viewDirection =
            methodParameters->viewpoint - pointCloud.points()[index].vector();
        if (viewDirection.squaredNorm() > 1.0e-24) {
            const double alignment = direction.dot(viewDirection);
            const bool shouldFlip = methodParameters->pointTowardViewpoint
                ? alignment < 0.0
                : alignment > 0.0;
            if (shouldFlip) {
                direction = -direction;
            }
        }
        result.normalField.addNormal(direction, inputNormal.confidence());
        if ((index & 127U) == 0U) {
            reporter.report(
                ProcessingStage::OrientingNormals,
                static_cast<double>(index) / static_cast<double>(inputNormals.size()),
                "Orienting normals from viewpoint");
        }
    }

    result.normalField.setConsistentlyOriented(true);
    finalizeNormalDiagnostics(
        result.normalField,
        pointCloud.pointCount(),
        result.diagnostics);
    result.diagnostics.connectedComponentCount =
        result.normalField.validNormalCount() > 0U ? 1U : 0U;
    result.succeeded = true;
    reporter.report(ProcessingStage::OrientingNormals, 0.99, "Viewpoint orientation complete");
    return result;
}
