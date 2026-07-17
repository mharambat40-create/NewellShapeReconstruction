#ifndef NEWELL_MODEL_PROCESSING_NORMALS_NORMALMETHODREGISTRY_H
#define NEWELL_MODEL_PROCESSING_NORMALS_NORMALMETHODREGISTRY_H

#include "model/processing/normals/IFieldNormalGenerator.h"
#include "model/processing/normals/INormalEstimator.h"
#include "model/processing/normals/INormalOrienter.h"

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

class NormalMethodRegistry
{
public:
    [[nodiscard]] NormalMethodAvailability availability(
        NormalMethod method,
        const NormalProcessingContext &context) const;
    [[nodiscard]] NormalMethodRequirements requirements(NormalMethod method) const;
    [[nodiscard]] std::unique_ptr<INormalEstimator> createEstimator(NormalMethod method) const;
    [[nodiscard]] std::unique_ptr<INormalOrienter> createOrienter(NormalMethod method) const;
    [[nodiscard]] std::unique_ptr<IFieldNormalGenerator> createFieldGenerator(NormalMethod method) const;

    [[nodiscard]] static constexpr std::array<NormalMethod, 4> pointCloudEstimatorMethods()
    {
        return {
            NormalMethod::PcaKNearest,
            NormalMethod::PcaFixedRadius,
            NormalMethod::PcaMultiScale,
            NormalMethod::QuadraticSurfaceFit,
        };
    }
    [[nodiscard]] static std::string displayName(NormalMethod method);
    [[nodiscard]] static std::optional<NormalMethod> methodFromDisplayName(
        std::string_view displayName);
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_NORMALMETHODREGISTRY_H
