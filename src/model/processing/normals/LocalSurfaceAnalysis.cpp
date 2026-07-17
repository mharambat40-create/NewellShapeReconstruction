#include "model/processing/normals/LocalSurfaceAnalysis.h"

#include "model/geometry/PointCloud.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

bool hasOnlyFiniteCoordinates(const PointCloud &pointCloud)
{
    return std::all_of(
        pointCloud.points().begin(),
        pointCloud.points().end(),
        [](const Point3d &point) {
            return point.vector().allFinite();
        });
}

LocalPlaneEstimate estimateLocalPlane(
    const PointCloud &pointCloud,
    const std::vector<Neighbour> &neighbours,
    double degeneracyTolerance)
{
    LocalPlaneEstimate estimate;
    if (neighbours.size() < 3U || !std::isfinite(degeneracyTolerance) ||
        degeneracyTolerance <= 0.0) {
        return estimate;
    }

    for (const Neighbour &neighbour : neighbours) {
        if (neighbour.pointIndex >= pointCloud.pointCount() ||
            !pointCloud.points()[neighbour.pointIndex].vector().allFinite()) {
            return estimate;
        }
        estimate.centroid += pointCloud.points()[neighbour.pointIndex].vector();
    }
    estimate.centroid /= static_cast<double>(neighbours.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Neighbour &neighbour : neighbours) {
        const Eigen::Vector3d offset =
            pointCloud.points()[neighbour.pointIndex].vector() - estimate.centroid;
        covariance.noalias() += offset * offset.transpose();
    }
    covariance /= static_cast<double>(neighbours.size());
    if (!covariance.allFinite()) {
        return estimate;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return estimate;
    }

    estimate.eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double totalVariance = estimate.eigenvalues.sum();
    const double largestEigenvalue = estimate.eigenvalues.z();
    if (!std::isfinite(totalVariance) || largestEigenvalue <= degeneracyTolerance ||
        estimate.eigenvalues.y() <= largestEigenvalue * degeneracyTolerance) {
        return estimate;
    }

    estimate.normal = solver.eigenvectors().col(0);
    const double normalLength = estimate.normal.norm();
    if (!estimate.normal.allFinite() || normalLength <= degeneracyTolerance) {
        return estimate;
    }
    estimate.normal /= normalLength;
    estimate.surfaceVariation = std::clamp(
        estimate.eigenvalues.x() / std::max(totalVariance, std::numeric_limits<double>::epsilon()),
        0.0,
        1.0);
    estimate.confidence = std::clamp(1.0 - estimate.surfaceVariation, 0.0, 1.0);
    estimate.valid = true;
    return estimate;
}

void finalizeNormalDiagnostics(
    const NormalField &normalField,
    std::size_t requestedPointCount,
    NormalDiagnostics &diagnostics)
{
    diagnostics.requestedPointCount = requestedPointCount;
    diagnostics.validNormalCount = normalField.validNormalCount();
    diagnostics.invalidNormalCount =
        requestedPointCount >= diagnostics.validNormalCount
        ? requestedPointCount - diagnostics.validNormalCount
        : 0U;
    diagnostics.consistentlyOriented = normalField.consistentlyOriented();

    std::vector<double> confidenceValues;
    confidenceValues.reserve(normalField.validNormalCount());
    double confidenceSum = 0.0;
    for (const Normal3d &normal : normalField.normals()) {
        if (!normal.isValid()) {
            continue;
        }
        confidenceValues.push_back(normal.confidence());
        confidenceSum += normal.confidence();
    }
    if (confidenceValues.empty()) {
        return;
    }
    diagnostics.meanConfidence =
        confidenceSum / static_cast<double>(confidenceValues.size());
    const auto median = confidenceValues.begin() +
        static_cast<std::ptrdiff_t>(confidenceValues.size() / 2U);
    std::nth_element(confidenceValues.begin(), median, confidenceValues.end());
    diagnostics.medianConfidence = *median;
}
