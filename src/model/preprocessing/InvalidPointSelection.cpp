#include "model/preprocessing/InvalidPointSelection.h"

#include <algorithm>
#include <cmath>
#include <vector>

std::vector<std::size_t> selectSparseRegionPoints(
    const PointCloud &cloud,
    double radiusMax,
    int minimumNeighbourCount)
{
    std::vector<std::size_t> selectedIndices;

    if (cloud.empty() || radiusMax <= 0.0 || minimumNeighbourCount < 0) {
        return selectedIndices;
    }

    const double squaredRadius = radiusMax * radiusMax;
    const PointCloud::Container &points = cloud.points();
    selectedIndices.reserve(points.size());

    // O(n^2) is acceptable for the current prototype. Replace this with a KD-tree
    // or spatial hash before using it on large point clouds.
    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
        int neighbourCount = 0;
        const Eigen::Vector3d point = points[pointIndex].vector();

        for (std::size_t neighbourIndex = 0; neighbourIndex < points.size(); ++neighbourIndex) {
            if (neighbourIndex == pointIndex) {
                continue;
            }

            const double squaredDistance =
                (points[neighbourIndex].vector() - point).squaredNorm();
            if (squaredDistance <= squaredRadius) {
                ++neighbourCount;
            }
        }

        if (neighbourCount < minimumNeighbourCount) {
            selectedIndices.push_back(pointIndex);
        }
    }

    return selectedIndices;
}

PointCloud removePointIndices(
    const PointCloud &cloud,
    const std::vector<std::size_t> &selectedIndices)
{
    PointCloud filteredCloud;
    if (cloud.empty()) {
        return filteredCloud;
    }

    std::vector<std::size_t> normalizedIndices = selectedIndices;
    std::sort(normalizedIndices.begin(), normalizedIndices.end());
    normalizedIndices.erase(
        std::remove_if(
            normalizedIndices.begin(),
            normalizedIndices.end(),
            [&cloud](std::size_t index) { return index >= cloud.pointCount(); }),
        normalizedIndices.end());
    normalizedIndices.erase(
        std::unique(normalizedIndices.begin(), normalizedIndices.end()),
        normalizedIndices.end());

    filteredCloud.reserve(cloud.pointCount() - normalizedIndices.size());

    std::size_t selectedCursor = 0;
    for (std::size_t pointIndex = 0; pointIndex < cloud.pointCount(); ++pointIndex) {
        if (selectedCursor < normalizedIndices.size() &&
            normalizedIndices[selectedCursor] == pointIndex) {
            ++selectedCursor;
            continue;
        }

        filteredCloud.addPoint(cloud.points()[pointIndex]);
    }

    return filteredCloud;
}
