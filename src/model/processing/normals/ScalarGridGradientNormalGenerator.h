#ifndef NEWELL_MODEL_PROCESSING_NORMALS_SCALARGRIDGRADIENTNORMALGENERATOR_H
#define NEWELL_MODEL_PROCESSING_NORMALS_SCALARGRIDGRADIENTNORMALGENERATOR_H

#include "model/processing/normals/IFieldNormalGenerator.h"

class ScalarGridGradientNormalGenerator final : public IFieldNormalGenerator
{
public:
    [[nodiscard]] NormalMethod method() const override;
    [[nodiscard]] NormalEstimationResult generate(
        const FieldNormalInput &input,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const override;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_SCALARGRIDGRADIENTNORMALGENERATOR_H
