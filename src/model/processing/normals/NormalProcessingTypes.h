#ifndef NEWELL_MODEL_PROCESSING_NORMALS_NORMALPROCESSINGTYPES_H
#define NEWELL_MODEL_PROCESSING_NORMALS_NORMALPROCESSINGTYPES_H

#include "model/geometry/NormalField.h"

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <variant>
#include <vector>

enum class NormalMethod
{
    PcaKNearest,
    PcaFixedRadius,
    PcaMultiScale,
    QuadraticSurfaceFit,
    GraphOrientation,
    ViewpointOrientation,
    ImplicitFieldGradient,
    ScalarGridGradient,
};

enum class NormalMethodCategory
{
    Estimation,
    Orientation,
    FieldDerived,
};

enum class ExistingNormalRequirement
{
    NotRequired,
    Required,
};

enum class FieldRequirement
{
    None,
    ImplicitScalarField,
    VoxelScalarField,
};

struct NormalMethodRequirements
{
    NormalMethodCategory category = NormalMethodCategory::Estimation;
    ExistingNormalRequirement existingNormals = ExistingNormalRequirement::NotRequired;
    FieldRequirement field = FieldRequirement::None;
};

struct PcaKnnParameters
{
    std::size_t neighbourCount = 12U;
    double maximumSearchRadius = std::numeric_limits<double>::infinity();
    double degeneracyTolerance = 1.0e-12;
};

using NormalEstimationParameters = PcaKnnParameters;

struct PcaRadiusParameters
{
    double searchRadius = 1.0;
    std::size_t minimumNeighbours = 6U;
    std::size_t maximumNeighbours = 64U;
    double degeneracyTolerance = 1.0e-12;
};

struct MultiScalePcaParameters
{
    std::vector<std::size_t> neighbourCounts{8U, 16U, 32U, 64U};
    double curvatureWeight = 0.6;
    double stabilityWeight = 0.4;
    std::size_t minimumValidScales = 2U;
    double degeneracyTolerance = 1.0e-12;
};

struct QuadraticFitParameters
{
    std::size_t neighbourCount = 24U;
    double regularisation = 1.0e-8;
    double maximumConditionNumber = 1.0e8;
    double degeneracyTolerance = 1.0e-12;
};

enum class ComponentSeedOrientation
{
    PreserveInput,
    TowardViewpoint,
    AwayFromCloudCentroid,
};

struct GraphOrientationParameters
{
    std::size_t neighbourCount = 12U;
    double maximumPropagationAngleRadians = 1.3962634015954636;
    ComponentSeedOrientation seedOrientation = ComponentSeedOrientation::AwayFromCloudCentroid;
    Eigen::Vector3d viewpoint = Eigen::Vector3d::Zero();
};

struct ViewpointOrientationParameters
{
    Eigen::Vector3d viewpoint = Eigen::Vector3d::Zero();
    bool pointTowardViewpoint = true;
};

struct ImplicitGradientParameters
{
    double finiteDifferenceStep = 1.0e-3;
    double minimumGradientMagnitude = 1.0e-10;
    bool invertDirection = false;
};

struct ScalarGridGradientParameters
{
    double minimumGradientMagnitude = 1.0e-10;
    bool invertDirection = false;
    bool interpolateGradient = true;
};

using NormalMethodParameters = std::variant<
    PcaKnnParameters,
    PcaRadiusParameters,
    MultiScalePcaParameters,
    QuadraticFitParameters,
    GraphOrientationParameters,
    ViewpointOrientationParameters,
    ImplicitGradientParameters,
    ScalarGridGradientParameters>;

struct NormalDiagnostics
{
    std::size_t requestedPointCount = 0U;
    std::size_t validNormalCount = 0U;
    std::size_t invalidNormalCount = 0U;
    std::size_t connectedComponentCount = 0U;
    std::size_t rejectedNeighbourhoodCount = 0U;
    std::size_t illConditionedFitCount = 0U;
    std::size_t nearZeroGradientCount = 0U;
    double meanConfidence = 0.0;
    double medianConfidence = 0.0;
    bool consistentlyOriented = false;
    std::vector<std::size_t> selectedNeighbourCounts;
};

struct NormalEstimationResult
{
    bool succeeded = false;
    bool cancelled = false;
    NormalField normalField;
    NormalDiagnostics diagnostics;
    std::vector<std::size_t> invalidPointIndices;
    std::string errorMessage;
};

struct NormalOrientationResult
{
    bool succeeded = false;
    bool cancelled = false;
    NormalField normalField;
    NormalDiagnostics diagnostics;
    std::string errorMessage;
};

struct NormalProcessingContext
{
    bool hasPointCloud = false;
    bool hasValidNormals = false;
    bool hasImplicitField = false;
    bool hasScalarGrid = false;
};

struct NormalMethodAvailability
{
    bool available = false;
    std::string reason;
};

struct NormalProcessingConfiguration
{
    NormalMethod estimationMethod = NormalMethod::PcaKNearest;
    NormalMethodParameters estimationParameters = PcaKnnParameters{};
    std::optional<NormalMethod> orientationMethod;
    NormalMethodParameters orientationParameters = GraphOrientationParameters{};
};

#endif // NEWELL_MODEL_PROCESSING_NORMALS_NORMALPROCESSINGTYPES_H
