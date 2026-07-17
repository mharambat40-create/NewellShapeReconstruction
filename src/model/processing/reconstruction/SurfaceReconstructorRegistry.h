#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_SURFACERECONSTRUCTORREGISTRY_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_SURFACERECONSTRUCTORREGISTRY_H

#include "model/processing/reconstruction/ReconstructionTypes.h"

#include <memory>
#include <optional>
#include <string_view>

class ISurfaceReconstructor;

class SurfaceReconstructorRegistry
{
public:
    [[nodiscard]] std::unique_ptr<ISurfaceReconstructor> create(
        ReconstructionMethod method) const;
    [[nodiscard]] ReconstructionAvailability availability(
        ReconstructionMethod method) const;
    [[nodiscard]] ReconstructionRequirements requirements(
        ReconstructionMethod method) const;

    [[nodiscard]] static std::optional<ReconstructionMethod> methodFromDisplayName(
        std::string_view displayName);
    [[nodiscard]] static std::string_view displayName(ReconstructionMethod method);
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_SURFACERECONSTRUCTORREGISTRY_H
