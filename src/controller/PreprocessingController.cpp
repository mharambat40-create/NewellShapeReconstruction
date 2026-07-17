#include "controller/PreprocessingController.h"

PreprocessingPipelineResult PreprocessingController::process(
    const PointCloud &pointCloud,
    const PreprocessingPipelineParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    return pipeline_.execute(pointCloud, parameters, progressCallback, cancellationToken);
}
