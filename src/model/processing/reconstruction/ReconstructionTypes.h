#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RECONSTRUCTIONTYPES_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RECONSTRUCTIONTYPES_H

#include <cstddef>
#include <string>
#include <variant>

class INeighbourSearch;
class NormalField;
class PointCloud;
class ScalarGrid3D;

enum class ReconstructionMethod
{
    Delaunay25D,
    AlphaShapes,
    BallPivoting,
    ScreenedPoisson,
    Rbf,
    MarchingCubes,
    Voxel,
    GreedyProjection,
};

enum class NormalRequirement
{
    NotUsed,
    Optional,
    Required,
    RequiredAndOriented,
};

enum class InputRepresentation
{
    PointCloud,
    OrientedPointCloud,
    ScalarField,
    VoxelGrid,
};

struct ReconstructionRequirements
{
    NormalRequirement normals = NormalRequirement::NotUsed;
    InputRepresentation input = InputRepresentation::PointCloud;
};

enum class ProjectionPlane
{
    XY,
    XZ,
    YZ,
    BestFitPlane,
};

enum class PlanarReconstructionMode
{
    Planar2D,
    HeightField25D,
};

struct Delaunay25DParameters
{
    ProjectionPlane projectionPlane = ProjectionPlane::BestFitPlane;
    PlanarReconstructionMode mode = PlanarReconstructionMode::HeightField25D;
    double planarityTolerance = 0.02;
    double maximumEdgeLength = 0.0;
    double geometricTolerance = 1.0e-9;
};

struct AlphaShapeParameters
{
    double alpha = 1.0;
    bool automaticAlpha = true;
    double automaticAlphaFactor = 4.0;
    bool regularised = true;
    ProjectionPlane projectionPlane = ProjectionPlane::BestFitPlane;
    PlanarReconstructionMode mode = PlanarReconstructionMode::HeightField25D;
    double planarityTolerance = 0.02;
    bool keepLargestComponentOnly = false;
    double minimumComponentArea = 0.0;
    double geometricTolerance = 1.0e-9;
};

struct BallPivotingParameters
{
    double ballRadius = 0.5;
    double geometricTolerance = 1.0e-9;
};

// Compatibility alias for the first reconstruction slice and its tests.
using ReconstructionParameters = BallPivotingParameters;

struct GreedyProjectionParameters
{
    double searchRadius = 1.0;
    std::size_t maximumNeighbours = 32U;
    double maximumSurfaceAngleRadians = 0.7853981633974483;
    double minimumTriangleAngleRadians = 0.17453292519943295;
    double maximumTriangleAngleRadians = 2.6179938779914944;
    double geometricTolerance = 1.0e-9;
};

struct VoxelReconstructionParameters
{
    double voxelSize = 0.25;
    int paddingVoxels = 2;
    double isoValue = 0.5;
    std::size_t maximumVoxelCount = 16U * 1024U * 1024U;
    bool closeSmallGaps = true;
};

struct MarchingCubesParameters
{
    double isoValue = 0.0;
    double geometricTolerance = 1.0e-9;
};

enum class RbfKernel
{
    WendlandC2,
    Gaussian,
    Multiquadric,
};

struct RbfParameters
{
    RbfKernel kernel = RbfKernel::WendlandC2;
    double supportRadius = 1.0;
    double regularisation = 1.0e-8;
    std::size_t maximumControlPoints = 128U;
    double gridSpacing = 0.25;
    std::size_t maximumVoxelCount = 8U * 1024U * 1024U;
    std::size_t maximumFieldEvaluations = 100U * 1024U * 1024U;
};

struct ScreenedPoissonParameters
{
    int depth = 8;
    double pointWeight = 4.0;
    double samplesPerNode = 1.5;
    int solverIterations = 8;
};

using ReconstructionMethodParameters = std::variant<
    Delaunay25DParameters,
    AlphaShapeParameters,
    BallPivotingParameters,
    GreedyProjectionParameters,
    VoxelReconstructionParameters,
    MarchingCubesParameters,
    RbfParameters,
    ScreenedPoissonParameters>;

struct ReconstructionInput
{
    const PointCloud *pointCloud = nullptr;
    const NormalField *normalField = nullptr;
    const INeighbourSearch *neighbourSearch = nullptr;
    const ScalarGrid3D *scalarGrid = nullptr;
};

struct ReconstructionAvailability
{
    bool available = false;
    std::string reason;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_RECONSTRUCTIONTYPES_H
