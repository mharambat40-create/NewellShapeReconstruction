#ifndef NEWELL_MODEL_PROCESSING_NORMALS_INORMALESTIMATOR_H
#define NEWELL_MODEL_PROCESSING_NORMALS_INORMALESTIMATOR_H

#include "model/geometry/NormalField.h"
#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/normals/NormalProcessingTypes.h"

class INeighbourSearch;
class PointCloud;

class INormalEstimator
{
public:
    virtual ~INormalEstimator() = default;

    [[nodiscard]] virtual NormalMethod method() const = 0;
    [[nodiscard]] virtual NormalEstimationResult estimate(
        const PointCloud &pointCloud,
        const INeighbourSearch &neighbourSearch,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const = 0;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_INORMALESTIMATOR_H
