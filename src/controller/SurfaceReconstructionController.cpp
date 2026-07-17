#include "controller/SurfaceReconstructionController.h"

ReconstructionPipelineResult SurfaceReconstructionController::reconstruct(
    const PointCloud &pointCloud,
    const ReconstructionPipelineParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    return pipeline_.execute(pointCloud, parameters, progressCallback, cancellationToken);
}
