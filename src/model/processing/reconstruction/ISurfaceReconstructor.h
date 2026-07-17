#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_ISURFACERECONSTRUCTOR_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_ISURFACERECONSTRUCTOR_H

#include "model/geometry/TriangleMesh.h"
#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/reconstruction/ReconstructionTypes.h"
#include "model/processing/reconstruction/common/MeshValidation.h"

#include <string>

struct BallPivotingPhaseTimings
{
    double preparationMilliseconds = 0.0;
    double seedSearchMilliseconds = 0.0;
    double frontPropagationMilliseconds = 0.0;
    double meshValidationMilliseconds = 0.0;
};

struct ReconstructionResult
{
    bool succeeded = false;
    bool cancelled = false;
    TriangleMesh mesh;
    MeshDiagnostics diagnostics;
    BallPivotingPhaseTimings timings;
    std::string errorMessage;
};

class ISurfaceReconstructor
{
public:
    virtual ~ISurfaceReconstructor() = default;

    [[nodiscard]] virtual ReconstructionMethod method() const = 0;
    [[nodiscard]] virtual ReconstructionRequirements requirements() const = 0;
    [[nodiscard]] virtual ReconstructionResult reconstruct(
        const ReconstructionInput &input,
        const ReconstructionMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const = 0;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_ISURFACERECONSTRUCTOR_H
