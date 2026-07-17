#ifndef NEWELL_MODEL_PIPELINE_PREPROCESSINGPIPELINE_H
#define NEWELL_MODEL_PIPELINE_PREPROCESSINGPIPELINE_H

#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"

#include <cstddef>
#include <string>
#include <vector>

class PointCloud;

enum class PreprocessingOperation
{
    SelectSparsePoints,
    SelectPerfectDuplicates,
    SelectNearDuplicates,
};

struct PreprocessingPipelineParameters
{
    PreprocessingOperation operation = PreprocessingOperation::SelectSparsePoints;
    double radius = 0.0;
    int minimumNeighbourCount = 0;
    double distanceThreshold = 0.0;
};

struct PreprocessingPipelineResult
{
    bool succeeded = false;
    bool cancelled = false;
    std::vector<std::size_t> selectedIndices;
    std::string errorMessage;
};

class PreprocessingPipeline
{
public:
    [[nodiscard]] PreprocessingPipelineResult execute(
        const PointCloud &pointCloud,
        const PreprocessingPipelineParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

#endif // NEWELL_MODEL_PIPELINE_PREPROCESSINGPIPELINE_H
