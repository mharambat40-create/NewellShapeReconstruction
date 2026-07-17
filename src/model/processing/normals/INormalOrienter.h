#ifndef NEWELL_MODEL_PROCESSING_NORMALS_INORMALORIENTER_H
#define NEWELL_MODEL_PROCESSING_NORMALS_INORMALORIENTER_H

#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/normals/NormalProcessingTypes.h"

class INeighbourSearch;
class PointCloud;

class INormalOrienter
{
public:
    virtual ~INormalOrienter() = default;

    [[nodiscard]] virtual NormalMethod method() const = 0;
    [[nodiscard]] virtual NormalOrientationResult orient(
        const PointCloud &pointCloud,
        const NormalField &inputNormals,
        const INeighbourSearch &neighbourSearch,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const = 0;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_INORMALORIENTER_H
