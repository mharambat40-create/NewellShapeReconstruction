#ifndef NEWELL_MODEL_PROCESSING_PREPROCESSING_SELECTIONALGORITHMS_H
#define NEWELL_MODEL_PROCESSING_PREPROCESSING_SELECTIONALGORITHMS_H

#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"

#include <cstddef>
#include <string>
#include <vector>

class PointCloud;

struct PointSelectionResult
{
    bool succeeded = false;
    bool cancelled = false;
    std::vector<std::size_t> selectedIndices;
    std::string errorMessage;
};

class InvalidPointSelector
{
public:
    [[nodiscard]] PointSelectionResult selectSparsePoints(
        const PointCloud &pointCloud,
        double radius,
        int minimumNeighbourCount,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

class DuplicatePointSelector
{
public:
    [[nodiscard]] PointSelectionResult selectPerfectDuplicates(
        const PointCloud &pointCloud,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
    [[nodiscard]] PointSelectionResult selectNearDuplicates(
        const PointCloud &pointCloud,
        double distanceThreshold,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

#endif // NEWELL_MODEL_PROCESSING_PREPROCESSING_SELECTIONALGORITHMS_H
