#include "model/processing/reconstruction/common/ProjectionPlaneUtilities.h"

#include "model/geometry/PointCloud.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

ProjectionBasisResult makeProjectionBasis(
    const PointCloud &pointCloud,
    ProjectionPlane projectionPlane)
{
    ProjectionBasisResult result;
    if (pointCloud.pointCount() < 3U) {
        result.errorMessage = "A projection plane requires at least three points.";
        return result;
    }

    switch (projectionPlane) {
    case ProjectionPlane::XY:
        result.basis.firstAxis = Eigen::Vector3d::UnitX();
        result.basis.secondAxis = Eigen::Vector3d::UnitY();
        result.basis.normal = Eigen::Vector3d::UnitZ();
        break;
    case ProjectionPlane::XZ:
        result.basis.firstAxis = Eigen::Vector3d::UnitX();
        result.basis.secondAxis = Eigen::Vector3d::UnitZ();
        result.basis.normal = -Eigen::Vector3d::UnitY();
        break;
    case ProjectionPlane::YZ:
        result.basis.firstAxis = Eigen::Vector3d::UnitY();
        result.basis.secondAxis = Eigen::Vector3d::UnitZ();
        result.basis.normal = Eigen::Vector3d::UnitX();
        break;
    case ProjectionPlane::BestFitPlane: {
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const Point3d &point : pointCloud.points()) {
            centroid += point.vector();
        }
        centroid /= static_cast<double>(pointCloud.pointCount());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const Point3d &point : pointCloud.points()) {
            const Eigen::Vector3d offset = point.vector() - centroid;
            covariance.noalias() += offset * offset.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success) {
            result.errorMessage = "Best-fit projection PCA failed.";
            return result;
        }
        result.basis.origin = centroid;
        result.basis.normal = solver.eigenvectors().col(0).normalized();
        result.basis.firstAxis = solver.eigenvectors().col(2).normalized();
        result.basis.secondAxis = result.basis.normal.cross(result.basis.firstAxis).normalized();
        if (!result.basis.firstAxis.allFinite() || !result.basis.secondAxis.allFinite() ||
            !result.basis.normal.allFinite()) {
            result.errorMessage = "Best-fit projection is degenerate.";
            return result;
        }
        break;
    }
    }

    result.succeeded = true;
    return result;
}

Eigen::Vector2d projectPoint(const Eigen::Vector3d &point, const ProjectionBasis &basis)
{
    const Eigen::Vector3d offset = point - basis.origin;
    return {offset.dot(basis.firstAxis), offset.dot(basis.secondAxis)};
}

PlanarProjectionResult computePlanarProjection(
    const PointCloud &pointCloud,
    double planarityTolerance)
{
    PlanarProjectionResult result;
    if (pointCloud.pointCount() < 3U) {
        result.errorMessage = "Planar reconstruction requires at least three points.";
        return result;
    }
    if (!std::isfinite(planarityTolerance) || planarityTolerance < 0.0 ||
        planarityTolerance >= 1.0) {
        result.errorMessage = "Planarity tolerance must be finite and in the range [0, 1).";
        return result;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const Point3d &point : pointCloud.points()) {
        if (!point.vector().allFinite()) {
            result.errorMessage = "The point cloud contains non-finite coordinates.";
            return result;
        }
        centroid += point.vector();
    }
    centroid /= static_cast<double>(pointCloud.pointCount());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Point3d &point : pointCloud.points()) {
        const Eigen::Vector3d offset = point.vector() - centroid;
        covariance.noalias() += offset * offset.transpose();
    }
    covariance /= static_cast<double>(pointCloud.pointCount());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        result.errorMessage = "Planar projection PCA failed.";
        return result;
    }

    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double totalVariance = eigenvalues.sum();
    const double scaleEpsilon = std::numeric_limits<double>::epsilon() *
        std::max(1.0, totalVariance) * 64.0;
    if (totalVariance <= scaleEpsilon || eigenvalues.y() <= scaleEpsilon) {
        result.errorMessage = "The point cloud is coincident or collinear and has no stable projection plane.";
        return result;
    }

    PlanarProjection projection;
    projection.basis.origin = centroid;
    projection.basis.normal = solver.eigenvectors().col(0).normalized();
    projection.basis.firstAxis = solver.eigenvectors().col(2).normalized();
    projection.basis.secondAxis = projection.basis.normal.cross(projection.basis.firstAxis).normalized();
    projection.eigenvalues = eigenvalues;
    projection.planarityIndicator = eigenvalues.x() / totalVariance;
    if (!projection.basis.firstAxis.allFinite() || !projection.basis.secondAxis.allFinite() ||
        !projection.basis.normal.allFinite()) {
        result.errorMessage = "The PCA projection basis is degenerate.";
        return result;
    }
    if (projection.planarityIndicator > planarityTolerance) {
        result.errorMessage =
            "The point cloud is too volumetric for planar reconstruction (planarity indicator " +
            std::to_string(projection.planarityIndicator) + " exceeds tolerance " +
            std::to_string(planarityTolerance) + ").";
        return result;
    }

    projection.projectedPoints.reserve(pointCloud.pointCount());
    projection.heights.reserve(pointCloud.pointCount());
    for (const Point3d &point : pointCloud.points()) {
        const Eigen::Vector3d offset = point.vector() - centroid;
        projection.projectedPoints.emplace_back(
            offset.dot(projection.basis.firstAxis),
            offset.dot(projection.basis.secondAxis));
        projection.heights.push_back(offset.dot(projection.basis.normal));
    }

    result.succeeded = true;
    result.projection = std::move(projection);
    return result;
}

Eigen::Vector3d reconstructProjectedPoint(
    const PlanarProjection &projection,
    std::size_t pointIndex,
    PlanarReconstructionMode mode)
{
    const Eigen::Vector2d &projected = projection.projectedPoints.at(pointIndex);
    const double height = mode == PlanarReconstructionMode::HeightField25D
        ? projection.heights.at(pointIndex)
        : 0.0;
    return projection.basis.origin + projection.basis.firstAxis * projected.x() +
        projection.basis.secondAxis * projected.y() + projection.basis.normal * height;
}
