#ifndef NEWELL_MODEL_PREPROCESSING_DUPLICATEPOINTSELECTION_H
#define NEWELL_MODEL_PREPROCESSING_DUPLICATEPOINTSELECTION_H

#include "model/geometry/PointCloud.h"

#include <cstddef>
#include <vector>

[[nodiscard]] std::vector<std::size_t> selectPerfectDuplicatePoints(
    const PointCloud &cloud);

[[nodiscard]] std::vector<std::size_t> selectNearDuplicatePoints(
    const PointCloud &cloud,
    double distanceThreshold);

#endif // NEWELL_MODEL_PREPROCESSING_DUPLICATEPOINTSELECTION_H
