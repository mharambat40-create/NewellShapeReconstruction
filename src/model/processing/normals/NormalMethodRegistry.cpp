#include "model/processing/normals/NormalMethodRegistry.h"

#include "model/processing/normals/GraphNormalOrienter.h"
#include "model/processing/normals/ImplicitFieldGradientNormalGenerator.h"
#include "model/processing/normals/MultiScalePcaNormalEstimator.h"
#include "model/processing/normals/PcaKnnNormalEstimator.h"
#include "model/processing/normals/PcaRadiusNormalEstimator.h"
#include "model/processing/normals/QuadraticSurfaceNormalEstimator.h"
#include "model/processing/normals/ScalarGridGradientNormalGenerator.h"
#include "model/processing/normals/ViewpointNormalOrienter.h"

NormalMethodAvailability NormalMethodRegistry::availability(
    NormalMethod method,
    const NormalProcessingContext &context) const
{
    const NormalMethodRequirements methodRequirements = requirements(method);
    if (methodRequirements.category == NormalMethodCategory::Estimation &&
        !context.hasPointCloud) {
        return {false, "Unavailable: Load a valid point cloud first."};
    }
    if (methodRequirements.existingNormals == ExistingNormalRequirement::Required &&
        !context.hasValidNormals) {
        return {false, "Unavailable: Compute or load a valid normal field first."};
    }
    if (methodRequirements.field == FieldRequirement::ImplicitScalarField &&
        !context.hasImplicitField) {
        return {false, "Unavailable: No implicit scalar field is available."};
    }
    if (methodRequirements.field == FieldRequirement::VoxelScalarField &&
        !context.hasScalarGrid) {
        return {false, "Unavailable: No voxel or scalar grid is available."};
    }
    return {true, {}};
}

NormalMethodRequirements NormalMethodRegistry::requirements(NormalMethod method) const
{
    switch (method) {
    case NormalMethod::PcaKNearest:
    case NormalMethod::PcaFixedRadius:
    case NormalMethod::PcaMultiScale:
    case NormalMethod::QuadraticSurfaceFit:
        return {
            NormalMethodCategory::Estimation,
            ExistingNormalRequirement::NotRequired,
            FieldRequirement::None,
        };
    case NormalMethod::GraphOrientation:
    case NormalMethod::ViewpointOrientation:
        return {
            NormalMethodCategory::Orientation,
            ExistingNormalRequirement::Required,
            FieldRequirement::None,
        };
    case NormalMethod::ImplicitFieldGradient:
        return {
            NormalMethodCategory::FieldDerived,
            ExistingNormalRequirement::NotRequired,
            FieldRequirement::ImplicitScalarField,
        };
    case NormalMethod::ScalarGridGradient:
        return {
            NormalMethodCategory::FieldDerived,
            ExistingNormalRequirement::NotRequired,
            FieldRequirement::VoxelScalarField,
        };
    }
    return {};
}

std::unique_ptr<INormalEstimator> NormalMethodRegistry::createEstimator(NormalMethod method) const
{
    switch (method) {
    case NormalMethod::PcaKNearest:
        return std::make_unique<PcaKnnNormalEstimator>();
    case NormalMethod::PcaFixedRadius:
        return std::make_unique<PcaRadiusNormalEstimator>();
    case NormalMethod::PcaMultiScale:
        return std::make_unique<MultiScalePcaNormalEstimator>();
    case NormalMethod::QuadraticSurfaceFit:
        return std::make_unique<QuadraticSurfaceNormalEstimator>();
    default:
        return nullptr;
    }
}

std::unique_ptr<INormalOrienter> NormalMethodRegistry::createOrienter(NormalMethod method) const
{
    switch (method) {
    case NormalMethod::GraphOrientation:
        return std::make_unique<GraphNormalOrienter>();
    case NormalMethod::ViewpointOrientation:
        return std::make_unique<ViewpointNormalOrienter>();
    default:
        return nullptr;
    }
}

std::unique_ptr<IFieldNormalGenerator> NormalMethodRegistry::createFieldGenerator(
    NormalMethod method) const
{
    switch (method) {
    case NormalMethod::ImplicitFieldGradient:
        return std::make_unique<ImplicitFieldGradientNormalGenerator>();
    case NormalMethod::ScalarGridGradient:
        return std::make_unique<ScalarGridGradientNormalGenerator>();
    default:
        return nullptr;
    }
}

std::string NormalMethodRegistry::displayName(NormalMethod method)
{
    switch (method) {
    case NormalMethod::PcaKNearest:
        return "PCA local plane — k-nearest neighbours";
    case NormalMethod::PcaFixedRadius:
        return "PCA local plane — fixed radius";
    case NormalMethod::PcaMultiScale:
        return "PCA multi-scale";
    case NormalMethod::QuadraticSurfaceFit:
        return "Quadratic local surface fitting";
    default:
        return {};
    }
}

std::optional<NormalMethod> NormalMethodRegistry::methodFromDisplayName(
    std::string_view candidate)
{
    for (const NormalMethod method : pointCloudEstimatorMethods()) {
        if (displayName(method) == candidate) {
            return method;
        }
    }
    return std::nullopt;
}
