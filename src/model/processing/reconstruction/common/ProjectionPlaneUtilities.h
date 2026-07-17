#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_PROJECTIONPLANEUTILITIES_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_PROJECTIONPLANEUTILITIES_H

#include "model/processing/reconstruction/ReconstructionTypes.h"

#include <Eigen/Core>

#include <string>
#include <vector>

class PointCloud;

struct ProjectionBasis
{
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d firstAxis = Eigen::Vector3d::UnitX();
    Eigen::Vector3d secondAxis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

struct ProjectionBasisResult
{
    bool succeeded = false;
    ProjectionBasis basis;
    std::string errorMessage;
};

struct PlanarProjection
{
    ProjectionBasis basis;
    std::vector<Eigen::Vector2d> projectedPoints;
    std::vector<double> heights;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    double planarityIndicator = 0.0;
};

struct PlanarProjectionResult
{
    bool succeeded = false;
    PlanarProjection projection;
    std::string errorMessage;
};

[[nodiscard]] ProjectionBasisResult makeProjectionBasis(
    const PointCloud &pointCloud,
    ProjectionPlane projectionPlane);

[[nodiscard]] Eigen::Vector2d projectPoint(
    const Eigen::Vector3d &point,
    const ProjectionBasis &basis);

[[nodiscard]] PlanarProjectionResult computePlanarProjection(
    const PointCloud &pointCloud,
    double planarityTolerance);

[[nodiscard]] Eigen::Vector3d reconstructProjectedPoint(
    const PlanarProjection &projection,
    std::size_t pointIndex,
    PlanarReconstructionMode mode);

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_PROJECTIONPLANEUTILITIES_H
