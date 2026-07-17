#ifndef NEWELL_MODEL_PROCESSING_NORMALS_QUADRATICSURFACENORMALESTIMATOR_H
#define NEWELL_MODEL_PROCESSING_NORMALS_QUADRATICSURFACENORMALESTIMATOR_H

#include "model/processing/normals/INormalEstimator.h"

class QuadraticSurfaceNormalEstimator final : public INormalEstimator
{
public:
    [[nodiscard]] NormalMethod method() const override;
    [[nodiscard]] NormalEstimationResult estimate(
        const PointCloud &pointCloud,
        const INeighbourSearch &neighbourSearch,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_QUADRATICSURFACENORMALESTIMATOR_H
