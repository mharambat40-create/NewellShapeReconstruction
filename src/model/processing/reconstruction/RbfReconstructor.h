#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RBFRECONSTRUCTOR_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RBFRECONSTRUCTOR_H

#include "model/processing/reconstruction/ISurfaceReconstructor.h"

class RbfReconstructor final : public ISurfaceReconstructor
{
public:
    [[nodiscard]] ReconstructionMethod method() const override;
    [[nodiscard]] ReconstructionRequirements requirements() const override;
    [[nodiscard]] ReconstructionResult reconstruct(
        const ReconstructionInput &input,
        const ReconstructionMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RBFRECONSTRUCTOR_H
