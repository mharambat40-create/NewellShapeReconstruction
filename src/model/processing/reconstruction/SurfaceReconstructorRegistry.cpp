#include "model/processing/reconstruction/SurfaceReconstructorRegistry.h"

#include "model/processing/reconstruction/AlphaShapeReconstructor.h"
#include "model/processing/reconstruction/BallPivotingReconstructor.h"
#include "model/processing/reconstruction/Delaunay25DReconstructor.h"
#include "model/processing/reconstruction/GreedyProjectionReconstructor.h"
#include "model/processing/reconstruction/ISurfaceReconstructor.h"
#include "model/processing/reconstruction/MarchingCubesReconstructor.h"
#include "model/processing/reconstruction/RbfReconstructor.h"
#include "model/processing/reconstruction/VoxelReconstructor.h"

#include <memory>

std::unique_ptr<ISurfaceReconstructor> SurfaceReconstructorRegistry::create(
    ReconstructionMethod method) const
{
    if (!availability(method).available) {
        return nullptr;
    }
    switch (method) {
    case ReconstructionMethod::Delaunay25D:
        return std::make_unique<Delaunay25DReconstructor>();
    case ReconstructionMethod::AlphaShapes:
        return std::make_unique<AlphaShapeReconstructor>();
    case ReconstructionMethod::BallPivoting:
        return std::make_unique<BallPivotingReconstructor>();
    case ReconstructionMethod::Rbf:
        return std::make_unique<RbfReconstructor>();
    case ReconstructionMethod::MarchingCubes:
        return std::make_unique<MarchingCubesReconstructor>();
    case ReconstructionMethod::Voxel:
        return std::make_unique<VoxelReconstructor>();
    case ReconstructionMethod::GreedyProjection:
        return std::make_unique<GreedyProjectionReconstructor>();
    case ReconstructionMethod::ScreenedPoisson:
        return nullptr;
    }
    return nullptr;
}

ReconstructionAvailability SurfaceReconstructorRegistry::availability(
    ReconstructionMethod method) const
{
    switch (method) {
    case ReconstructionMethod::Delaunay25D:
    case ReconstructionMethod::AlphaShapes:
#if NEWELL_HAS_CGAL
        return {true, {}};
#else
        return {false, "CGAL is required. Install it with brew install cgal and reconfigure CMake."};
#endif
    case ReconstructionMethod::BallPivoting:
    case ReconstructionMethod::Rbf:
    case ReconstructionMethod::MarchingCubes:
    case ReconstructionMethod::Voxel:
    case ReconstructionMethod::GreedyProjection:
        return {true, {}};
    case ReconstructionMethod::ScreenedPoisson:
        return {false, "No screened octree Poisson backend is configured."};
    }
    return {false, "Unknown reconstruction method."};
}

ReconstructionRequirements SurfaceReconstructorRegistry::requirements(
    ReconstructionMethod method) const
{
    switch (method) {
    case ReconstructionMethod::Delaunay25D:
    case ReconstructionMethod::AlphaShapes:
    case ReconstructionMethod::Voxel:
        return {NormalRequirement::NotUsed, InputRepresentation::PointCloud};
    case ReconstructionMethod::BallPivoting:
        return {NormalRequirement::Required, InputRepresentation::PointCloud};
    case ReconstructionMethod::GreedyProjection:
        return {NormalRequirement::Required, InputRepresentation::PointCloud};
    case ReconstructionMethod::Rbf:
    case ReconstructionMethod::ScreenedPoisson:
        return {NormalRequirement::RequiredAndOriented, InputRepresentation::OrientedPointCloud};
    case ReconstructionMethod::MarchingCubes:
        return {NormalRequirement::NotUsed, InputRepresentation::ScalarField};
    }
    return {};
}

std::optional<ReconstructionMethod> SurfaceReconstructorRegistry::methodFromDisplayName(
    std::string_view displayName)
{
    for (const ReconstructionMethod method : {
             ReconstructionMethod::Delaunay25D,
             ReconstructionMethod::AlphaShapes,
             ReconstructionMethod::BallPivoting,
             ReconstructionMethod::ScreenedPoisson,
             ReconstructionMethod::Rbf,
             ReconstructionMethod::MarchingCubes,
             ReconstructionMethod::Voxel,
             ReconstructionMethod::GreedyProjection,
         }) {
        if (SurfaceReconstructorRegistry::displayName(method) == displayName) {
            return method;
        }
    }
    return std::nullopt;
}

std::string_view SurfaceReconstructorRegistry::displayName(ReconstructionMethod method)
{
    switch (method) {
    case ReconstructionMethod::Delaunay25D:
        return "Delaunay 2D / 2.5D Triangulation";
    case ReconstructionMethod::AlphaShapes:
        return "Alpha Shapes";
    case ReconstructionMethod::BallPivoting:
        return "Ball Pivoting Algorithm";
    case ReconstructionMethod::ScreenedPoisson:
        return "Screened Poisson Reconstruction";
    case ReconstructionMethod::Rbf:
        return "RBF Reconstruction";
    case ReconstructionMethod::MarchingCubes:
        return "Marching Cubes on Implicit Field";
    case ReconstructionMethod::Voxel:
        return "Voxel-based Reconstruction";
    case ReconstructionMethod::GreedyProjection:
        return "Greedy Projection Triangulation";
    }
    return "Unknown";
}
