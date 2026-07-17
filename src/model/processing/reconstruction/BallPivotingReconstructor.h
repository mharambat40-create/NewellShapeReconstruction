#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_BALLPIVOTINGRECONSTRUCTOR_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_BALLPIVOTINGRECONSTRUCTOR_H

#include "model/processing/reconstruction/ISurfaceReconstructor.h"

// Single-radius BPA. Empty-ball and edge-incidence tolerances are explicit in parameters.
class BallPivotingReconstructor final : public ISurfaceReconstructor
{
public:
    [[nodiscard]] ReconstructionMethod method() const override;
    [[nodiscard]] ReconstructionRequirements requirements() const override;
    [[nodiscard]] ReconstructionResult reconstruct(
        const ReconstructionInput &input,
        const ReconstructionMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;

    // Compatibility overload retained for the existing BPA tests and callers.
    [[nodiscard]] ReconstructionResult reconstruct(
        const PointCloud &pointCloud,
        const NormalField &normalField,
        const INeighbourSearch &neighbourSearch,
        const ReconstructionParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_BALLPIVOTINGRECONSTRUCTOR_H
