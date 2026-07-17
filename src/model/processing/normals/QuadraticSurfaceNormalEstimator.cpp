#include "model/processing/normals/QuadraticSurfaceNormalEstimator.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

NormalMethod QuadraticSurfaceNormalEstimator::method() const
{
    return NormalMethod::QuadraticSurfaceFit;
}

NormalEstimationResult QuadraticSurfaceNormalEstimator::estimate(
    const PointCloud &pointCloud,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalEstimationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<QuadraticFitParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Quadratic fitting received parameters for another normal method.";
        return result;
    }
    if (pointCloud.pointCount() < 7U || methodParameters->neighbourCount < 6U ||
        !std::isfinite(methodParameters->regularisation) || methodParameters->regularisation < 0.0 ||
        !std::isfinite(methodParameters->maximumConditionNumber) ||
        methodParameters->maximumConditionNumber <= 1.0 ||
        !std::isfinite(methodParameters->degeneracyTolerance) ||
        methodParameters->degeneracyTolerance <= 0.0) {
        result.errorMessage = "Quadratic-fit parameters are invalid.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "Quadratic fitting requires finite point coordinates.";
        return result;
    }

    result.normalField.reserve(pointCloud.pointCount());
    result.invalidPointIndices.reserve(pointCloud.pointCount() / 10U);
    reporter.report(ProcessingStage::EstimatingNormals, 0.0, "Fitting quadratic local surfaces");
    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        if ((pointIndex & 63U) == 0U) {
            reporter.report(
                ProcessingStage::EstimatingNormals,
                static_cast<double>(pointIndex) / static_cast<double>(pointCloud.pointCount()),
                "Fitting quadratic local surfaces");
        }

        const std::vector<Neighbour> neighbours = neighbourSearch.kNearest(
            pointCloud.points()[pointIndex].vector(),
            methodParameters->neighbourCount,
            pointIndex);
        const LocalPlaneEstimate plane = estimateLocalPlane(
            pointCloud,
            neighbours,
            methodParameters->degeneracyTolerance);
        if (!plane.valid || neighbours.size() < 6U) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.rejectedNeighbourhoodCount;
            continue;
        }

        const Eigen::Vector3d tangentU = plane.normal.unitOrthogonal().normalized();
        const Eigen::Vector3d tangentV = plane.normal.cross(tangentU).normalized();
        const Eigen::Vector3d query = pointCloud.points()[pointIndex].vector();
        const Eigen::Index sampleCount = static_cast<Eigen::Index>(neighbours.size());
        const Eigen::Index regularisationRows = methodParameters->regularisation > 0.0 ? 6 : 0;
        Eigen::MatrixXd design(sampleCount + regularisationRows, 6);
        Eigen::VectorXd values(sampleCount + regularisationRows);
        for (Eigen::Index row = 0; row < sampleCount; ++row) {
            const Eigen::Vector3d offset =
                pointCloud.points()[neighbours[static_cast<std::size_t>(row)].pointIndex].vector() - query;
            const double u = offset.dot(tangentU);
            const double v = offset.dot(tangentV);
            const double w = offset.dot(plane.normal);
            design.row(row) << u * u, u * v, v * v, u, v, 1.0;
            values[row] = w;
        }
        if (regularisationRows > 0) {
            design.bottomRows<6>() =
                std::sqrt(methodParameters->regularisation) * Eigen::Matrix<double, 6, 6>::Identity();
            values.tail<6>().setZero();
        }

        const Eigen::JacobiSVD<Eigen::MatrixXd> decomposition(
            design,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::VectorXd singularValues = decomposition.singularValues();
        const double smallestSingularValue = singularValues.size() == 6 ? singularValues[5] : 0.0;
        const double conditionNumber = smallestSingularValue > methodParameters->degeneracyTolerance
            ? singularValues[0] / smallestSingularValue
            : std::numeric_limits<double>::infinity();
        if (decomposition.rank() < 6 || !std::isfinite(conditionNumber) ||
            conditionNumber > methodParameters->maximumConditionNumber) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.illConditionedFitCount;
            continue;
        }

        const Eigen::VectorXd coefficients = decomposition.solve(values);
        if (!coefficients.allFinite()) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.illConditionedFitCount;
            continue;
        }
        Eigen::Vector3d normal =
            -coefficients[3] * tangentU - coefficients[4] * tangentV + plane.normal;
        const double normalLength = normal.norm();
        if (!normal.allFinite() || normalLength <= methodParameters->degeneracyTolerance) {
            result.normalField.addInvalidNormal();
            result.invalidPointIndices.push_back(pointIndex);
            ++result.diagnostics.illConditionedFitCount;
            continue;
        }
        normal /= normalLength;

        const Eigen::VectorXd residual =
            design.topRows(sampleCount) * coefficients - values.head(sampleCount);
        const double rootMeanSquareResidual =
            std::sqrt(residual.squaredNorm() / static_cast<double>(sampleCount));
        const double neighbourhoodScale = std::sqrt(
            std::max(plane.eigenvalues.z(), methodParameters->degeneracyTolerance));
        const double fitConfidence = 1.0 /
            (1.0 + rootMeanSquareResidual / neighbourhoodScale);
        result.normalField.addNormal(
            normal,
            std::clamp(plane.confidence * fitConfidence, 0.0, 1.0));
    }

    finalizeNormalDiagnostics(result.normalField, pointCloud.pointCount(), result.diagnostics);
    if (result.normalField.validNormalCount() == 0U) {
        result.errorMessage = "No reliable quadratic surface normals could be fitted.";
        return result;
    }
    result.succeeded = true;
    reporter.report(ProcessingStage::EstimatingNormals, 0.99, "Quadratic surface normals ready");
    return result;
}
