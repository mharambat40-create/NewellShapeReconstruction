#ifndef NEWELL_CONTROLLER_SURFACERECONSTRUCTIONCONTROLLER_H
#define NEWELL_CONTROLLER_SURFACERECONSTRUCTIONCONTROLLER_H

#include "model/pipeline/ReconstructionPipeline.h"

class PointCloud;

class SurfaceReconstructionController
{
public:
    [[nodiscard]] ReconstructionPipelineResult reconstruct(
        const PointCloud &pointCloud,
        const ReconstructionPipelineParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;

private:
    ReconstructionPipeline pipeline_;
};

#endif // NEWELL_CONTROLLER_SURFACERECONSTRUCTIONCONTROLLER_H
