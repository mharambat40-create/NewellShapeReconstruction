#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_MARCHINGCUBESRECONSTRUCTOR_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_MARCHINGCUBESRECONSTRUCTOR_H

#include "model/processing/reconstruction/ISurfaceReconstructor.h"

class ScalarGrid3D;

class MarchingCubesReconstructor final : public ISurfaceReconstructor
{
public:
    [[nodiscard]] ReconstructionMethod method() const override;
    [[nodiscard]] ReconstructionRequirements requirements() const override;
    [[nodiscard]] ReconstructionResult reconstruct(
        const ReconstructionInput &input,
        const ReconstructionMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;

    [[nodiscard]] ReconstructionResult extract(
        const ScalarGrid3D &grid,
        const MarchingCubesParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_MARCHINGCUBESRECONSTRUCTOR_H
