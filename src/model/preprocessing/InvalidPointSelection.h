#ifndef NEWELL_MODEL_PREPROCESSING_INVALIDPOINTSELECTION_H
#define NEWELL_MODEL_PREPROCESSING_INVALIDPOINTSELECTION_H

#include "model/geometry/PointCloud.h"

#include <cstddef>
#include <vector>

[[nodiscard]] std::vector<std::size_t> selectSparseRegionPoints(
    const PointCloud &cloud,
    double radiusMax,
    int minimumNeighbourCount);

[[nodiscard]] PointCloud removePointIndices(
    const PointCloud &cloud,
    const std::vector<std::size_t> &selectedIndices);

#endif // NEWELL_MODEL_PREPROCESSING_INVALIDPOINTSELECTION_H
