#ifndef NEWELL_MODEL_PROCESSING_NORMALS_GRAPHNORMALORIENTER_H
#define NEWELL_MODEL_PROCESSING_NORMALS_GRAPHNORMALORIENTER_H

#include "model/processing/normals/INormalOrienter.h"

class GraphNormalOrienter final : public INormalOrienter
{
public:
    [[nodiscard]] NormalMethod method() const override;
    [[nodiscard]] NormalOrientationResult orient(
        const PointCloud &pointCloud,
        const NormalField &inputNormals,
        const INeighbourSearch &neighbourSearch,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_GRAPHNORMALORIENTER_H
