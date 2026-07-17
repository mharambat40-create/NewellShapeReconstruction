#ifndef NEWELL_MODEL_PROCESSING_NORMALS_LOCALSURFACEANALYSIS_H
#define NEWELL_MODEL_PROCESSING_NORMALS_LOCALSURFACEANALYSIS_H

#include "model/processing/normals/NormalProcessingTypes.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <Eigen/Core>

#include <vector>

class PointCloud;

struct LocalPlaneEstimate
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    double confidence = 0.0;
    double surfaceVariation = 1.0;
    bool valid = false;
};

[[nodiscard]] bool hasOnlyFiniteCoordinates(const PointCloud &pointCloud);

[[nodiscard]] LocalPlaneEstimate estimateLocalPlane(
    const PointCloud &pointCloud,
    const std::vector<Neighbour> &neighbours,
    double degeneracyTolerance);

void finalizeNormalDiagnostics(
    const NormalField &normalField,
    std::size_t requestedPointCount,
    NormalDiagnostics &diagnostics);

#endif // NEWELL_MODEL_PROCESSING_NORMALS_LOCALSURFACEANALYSIS_H
