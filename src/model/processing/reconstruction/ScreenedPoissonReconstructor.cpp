#include "model/processing/reconstruction/ScreenedPoissonReconstructor.h"

ReconstructionMethod ScreenedPoissonReconstructor::method() const
{
    return ReconstructionMethod::ScreenedPoisson;
}

ReconstructionRequirements ScreenedPoissonReconstructor::requirements() const
{
    return {NormalRequirement::RequiredAndOriented, InputRepresentation::OrientedPointCloud};
}

ReconstructionResult ScreenedPoissonReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    (void)input;
    (void)parameters;
    (void)progressCallback;
    (void)cancellationToken;
    ReconstructionResult result;
    result.errorMessage =
        "Screened Poisson is unavailable: no screened octree Poisson backend is configured.";
    return result;
}
