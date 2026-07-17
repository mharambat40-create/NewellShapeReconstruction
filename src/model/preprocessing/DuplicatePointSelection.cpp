#include "model/preprocessing/DuplicatePointSelection.h"

#include <algorithm>
#include <map>
#include <tuple>
#include <vector>

std::vector<std::size_t> selectPerfectDuplicatePoints(const PointCloud &cloud)
{
    std::vector<std::size_t> selectedIndices;

    if (cloud.empty()) {
        return selectedIndices;
    }

    using CoordinateKey = std::tuple<double, double, double>;
    std::map<CoordinateKey, std::size_t> representativeIndices;
    representativeIndices.clear();

    selectedIndices.reserve(cloud.pointCount() / 2U);
    const PointCloud::Container &points = cloud.points();

    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        const Point3d &point = points[pointIndex];
        const CoordinateKey coordinateKey(point.x(), point.y(), point.z());
        const auto [it, inserted] = representativeIndices.emplace(coordinateKey, pointIndex);
        if (!inserted) {
            selectedIndices.push_back(pointIndex);
        }
    }

    return selectedIndices;
}

std::vector<std::size_t> selectNearDuplicatePoints(
    const PointCloud &cloud,
    double distanceThreshold)
{
    std::vector<std::size_t> selectedIndices;

    if (cloud.empty() || distanceThreshold <= 0.0) {
        return selectedIndices;
    }

    const double squaredDistanceThreshold = distanceThreshold * distanceThreshold;
    const PointCloud::Container &points = cloud.points();
    std::vector<std::size_t> representativeIndices;
    representativeIndices.reserve(points.size());
    selectedIndices.reserve(points.size() / 2U);

    // O(n^2) is acceptable for the current prototype. Replace this with a
    // spatial index before using it on large point clouds.
    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        const Eigen::Vector3d point = points[pointIndex].vector();

        bool matchesRepresentative = false;
        for (const std::size_t representativeIndex : representativeIndices) {
            const double squaredDistance =
                (points[representativeIndex].vector() - point).squaredNorm();
            if (squaredDistance <= squaredDistanceThreshold) {
                matchesRepresentative = true;
                break;
            }
        }

        if (matchesRepresentative) {
            selectedIndices.push_back(pointIndex);
        } else {
            representativeIndices.push_back(pointIndex);
        }
    }

    return selectedIndices;
}
