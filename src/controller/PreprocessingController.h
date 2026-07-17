#ifndef NEWELL_CONTROLLER_PREPROCESSINGCONTROLLER_H
#define NEWELL_CONTROLLER_PREPROCESSINGCONTROLLER_H

#include "model/pipeline/PreprocessingPipeline.h"

class PreprocessingController
{
public:
    [[nodiscard]] PreprocessingPipelineResult process(
        const PointCloud &pointCloud,
        const PreprocessingPipelineParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;

private:
    PreprocessingPipeline pipeline_;
};

#endif // NEWELL_CONTROLLER_PREPROCESSINGCONTROLLER_H
