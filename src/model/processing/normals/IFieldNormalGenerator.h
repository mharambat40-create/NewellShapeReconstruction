#ifndef NEWELL_MODEL_PROCESSING_NORMALS_IFIELDNORMALGENERATOR_H
#define NEWELL_MODEL_PROCESSING_NORMALS_IFIELDNORMALGENERATOR_H

#include "model/processing/common/CancellationToken.h"
#include "model/processing/common/ProcessingProgress.h"
#include "model/processing/normals/NormalProcessingTypes.h"

#include <Eigen/Core>

#include <optional>

class PointCloud;
class ScalarGrid3D;

class IImplicitField
{
public:
    virtual ~IImplicitField() = default;

    [[nodiscard]] virtual double value(const Eigen::Vector3d &position) const = 0;
    [[nodiscard]] virtual std::optional<Eigen::Vector3d> gradient(
        const Eigen::Vector3d &position) const
    {
        (void)position;
        return std::nullopt;
    }
};

struct FieldNormalInput
{
    const PointCloud *queryPoints = nullptr;
    const IImplicitField *implicitField = nullptr;
    const ScalarGrid3D *scalarGrid = nullptr;
};

class IFieldNormalGenerator
{
public:
    virtual ~IFieldNormalGenerator() = default;

    [[nodiscard]] virtual NormalMethod method() const = 0;
    [[nodiscard]] virtual NormalEstimationResult generate(
        const FieldNormalInput &input,
        const NormalMethodParameters &parameters,
        const ProgressCallback &progressCallback = {},
        const CancellationToken &cancellationToken = {}) const = 0;
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_IFIELDNORMALGENERATOR_H
