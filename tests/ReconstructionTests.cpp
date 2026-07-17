#include "model/geometry/NormalField.h"
#include "model/geometry/GeometryDocument.h"
#include "model/geometry/PointCloud.h"
#include "model/geometry/TriangleMesh.h"
#include "model/pipeline/ReconstructionPipeline.h"
#include "model/processing/normals/PcaKnnNormalEstimator.h"
#include "model/processing/reconstruction/BallPivotingReconstructor.h"
#include "model/processing/reconstruction/AlphaShapeReconstructor.h"
#include "model/processing/reconstruction/Delaunay25DReconstructor.h"
#include "model/processing/reconstruction/GreedyProjectionReconstructor.h"
#include "model/processing/reconstruction/MarchingCubesReconstructor.h"
#include "model/processing/reconstruction/RbfReconstructor.h"
#include "model/processing/reconstruction/SurfaceReconstructorRegistry.h"
#include "model/processing/reconstruction/VoxelReconstructor.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"
#include "model/processing/reconstruction/common/ProjectionPlaneUtilities.h"
#include "model/processing/spatial/KDTree.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

PointCloud makePlane(std::size_t sideLength, double spacing, bool noisy = false)
{
    PointCloud cloud;
    cloud.reserve(sideLength * sideLength);
    const double half = static_cast<double>(sideLength - 1U) * spacing * 0.5;
    for (std::size_t row = 0; row < sideLength; ++row) {
        for (std::size_t column = 0; column < sideLength; ++column) {
            const double x = static_cast<double>(column) * spacing - half;
            const double y = static_cast<double>(row) * spacing - half;
            const double z = noisy ? 0.01 * std::sin(1.7 * x) * std::cos(1.3 * y) : 0.0;
            cloud.addPoint(x, y, z);
        }
    }
    return cloud;
}

PointCloud makeSphere(std::size_t pointCount)
{
    PointCloud cloud;
    cloud.reserve(pointCount);
    const double goldenAngle = std::numbers::pi_v<double> * (3.0 - std::sqrt(5.0));
    for (std::size_t index = 0; index < pointCount; ++index) {
        const double y = 1.0 - 2.0 * (static_cast<double>(index) + 0.5) /
            static_cast<double>(pointCount);
        const double radial = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double angle = goldenAngle * static_cast<double>(index);
        cloud.addPoint(radial * std::cos(angle), y, radial * std::sin(angle));
    }
    return cloud;
}

NormalField normalsFromPositions(const PointCloud &cloud)
{
    NormalField normals;
    normals.reserve(cloud.pointCount());
    for (const Point3d &point : cloud.points()) {
        normals.addNormal(point.vector());
    }
    return normals;
}

NormalField constantNormals(std::size_t count, const Eigen::Vector3d &direction)
{
    NormalField normals;
    normals.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        normals.addNormal(direction);
    }
    return normals;
}

void requireConsistentMesh(const TriangleMesh &mesh)
{
    require(!mesh.empty(), "The reconstructed mesh should contain triangles.");
    require(mesh.hasValidIndices(), "Every triangle index should reference a valid, distinct vertex.");

    std::set<std::array<std::size_t, 3>> uniqueTriangles;
    std::map<std::array<std::size_t, 2>, std::size_t> edgeCounts;
    for (const Triangle &triangle : mesh.triangles()) {
        auto sortedTriangle = triangle.vertexIndices;
        std::sort(sortedTriangle.begin(), sortedTriangle.end());
        require(uniqueTriangles.insert(sortedTriangle).second, "The mesh should not contain duplicate triangles.");

        const Point3d &first = mesh.vertices()[triangle.vertexIndices[0]];
        const Point3d &second = mesh.vertices()[triangle.vertexIndices[1]];
        const Point3d &third = mesh.vertices()[triangle.vertexIndices[2]];
        require(
            (second.vector() - first.vector()).cross(third.vector() - first.vector()).norm() > 1.0e-10,
            "The mesh should not contain degenerate triangles.");

        for (std::array<std::size_t, 2> edge : {
                 std::array<std::size_t, 2>{triangle.vertexIndices[0], triangle.vertexIndices[1]},
                 std::array<std::size_t, 2>{triangle.vertexIndices[1], triangle.vertexIndices[2]},
                 std::array<std::size_t, 2>{triangle.vertexIndices[2], triangle.vertexIndices[0]}}) {
            std::sort(edge.begin(), edge.end());
            require(++edgeCounts[edge] <= 2U, "A manifold edge should have at most two incident faces.");
        }
    }
}

void testKDTreeQueries()
{
    PointCloud cloud;
    cloud.addPoint(0.0, 0.0, 0.0);
    cloud.addPoint(1.0, 0.0, 0.0);
    cloud.addPoint(2.0, 0.0, 0.0);
    cloud.addPoint(4.0, 0.0, 0.0);
    const KDTree tree(cloud);

    const auto nearest = tree.kNearest(Eigen::Vector3d(0.2, 0.0, 0.0), 2U);
    require(nearest.size() == 2U, "k-nearest search should return the requested number of points.");
    require(nearest[0].pointIndex == 0U && nearest[1].pointIndex == 1U, "k-nearest results should be distance ordered.");

    const auto radius = tree.radiusSearch(Eigen::Vector3d(1.0, 0.0, 0.0), 1.01, 1U);
    require(radius.size() == 2U, "Radius search should find both adjacent points while excluding the query index.");
    require(radius[0].pointIndex == 0U && radius[1].pointIndex == 2U, "Radius results should be deterministic.");
}

NormalEstimationResult estimateNormals(const PointCloud &cloud, std::size_t neighbours)
{
    const KDTree tree(cloud);
    NormalEstimationParameters parameters;
    parameters.neighbourCount = neighbours;
    const PcaKnnNormalEstimator estimator;
    return estimator.estimate(cloud, tree, parameters);
}

void testPcaPlaneNormals()
{
    const PointCloud cloud = makePlane(9U, 0.25);
    const NormalEstimationResult result = estimateNormals(cloud, 12U);
    require(result.succeeded, "PCA should estimate normals for a regular plane.");
    require(result.normalField.validNormalCount() == cloud.pointCount(), "Every planar point should receive a valid normal.");
    for (const Normal3d &normal : result.normalField.normals()) {
        require(std::abs(normal.vector().norm() - 1.0) < 1.0e-10, "PCA normals should be unit length.");
        require(std::abs(normal.z()) > 0.999, "Planar PCA normals should align with the plane axis.");
    }
}

void testPcaNoisyPlaneNormals()
{
    const PointCloud cloud = makePlane(11U, 0.2, true);
    const NormalEstimationResult result = estimateNormals(cloud, 16U);
    require(result.succeeded, "PCA should handle a moderately noisy plane.");
    for (const Normal3d &normal : result.normalField.normals()) {
        require(normal.isValid(), "The noisy plane should still produce valid normals.");
        require(std::abs(normal.z()) > 0.98, "Noisy-plane normals should remain close to the plane axis.");
    }
}

void testPcaSphereNormals()
{
    const PointCloud cloud = makeSphere(240U);
    const NormalEstimationResult result = estimateNormals(cloud, 16U);
    require(result.succeeded, "PCA should estimate normals for a sampled sphere.");
    require(result.normalField.validNormalCount() > cloud.pointCount() * 9U / 10U, "Most sphere normals should be valid.");
    for (std::size_t index = 0; index < cloud.pointCount(); ++index) {
        const Normal3d &normal = result.normalField.normal(index);
        if (normal.isValid()) {
            require(
                std::abs(normal.vector().dot(cloud.points()[index].vector().normalized())) > 0.9,
                "Sphere PCA normals should align with radial directions up to sign.");
        }
    }
}

void testPlanarProjectionAndPlanarityRejection()
{
    const PointCloud noisyPlane = makePlane(9U, 0.25, true);
    const PlanarProjectionResult projection = computePlanarProjection(noisyPlane, 0.02);
    require(projection.succeeded, "A mildly varying height field should pass the planar PCA test.");
    require(projection.projection.planarityIndicator < 0.02, "The planar indicator should remain below its tolerance.");

    for (std::size_t index = 0; index < noisyPlane.pointCount(); ++index) {
        const Eigen::Vector3d planar = reconstructProjectedPoint(
            projection.projection,
            index,
            PlanarReconstructionMode::Planar2D);
        require(
            std::abs((planar - projection.projection.basis.origin)
                         .dot(projection.projection.basis.normal)) < 1.0e-10,
            "The 2D mode should place every vertex on the PCA plane.");
        const Eigen::Vector3d heightField = reconstructProjectedPoint(
            projection.projection,
            index,
            PlanarReconstructionMode::HeightField25D);
        require(
            (heightField - noisyPlane.points()[index].vector()).norm() < 1.0e-10,
            "The 2.5D mode should preserve each point's PCA height.");
    }

    const PlanarProjectionResult volumetric = computePlanarProjection(makeSphere(120U), 0.02);
    require(!volumetric.succeeded, "A spherical point cloud should be rejected as too volumetric.");
    require(
        volumetric.errorMessage.find("too volumetric") != std::string::npos,
        "The planarity rejection should explain why the input cannot be projected.");
}

#if NEWELL_HAS_CGAL
void testCgalDelaunayModesAndPlanarity()
{
    const PointCloud cloud = makePlane(8U, 0.3, true);
    Delaunay25DParameters parameters;
    parameters.planarityTolerance = 0.02;
    parameters.mode = PlanarReconstructionMode::Planar2D;
    const ReconstructionResult planar = Delaunay25DReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &cloud},
        parameters);
    require(planar.succeeded, "CGAL Delaunay 2D should reconstruct a near-planar grid.");
    requireConsistentMesh(planar.mesh);

    const PlanarProjectionResult projection = computePlanarProjection(cloud, parameters.planarityTolerance);
    require(projection.succeeded, "The Delaunay test cloud should have a stable PCA plane.");
    for (const Point3d &vertex : planar.mesh.vertices()) {
        require(
            std::abs((vertex.vector() - projection.projection.basis.origin)
                         .dot(projection.projection.basis.normal)) < 1.0e-10,
            "Delaunay 2D vertices should be coplanar.");
    }

    parameters.mode = PlanarReconstructionMode::HeightField25D;
    const ReconstructionResult heightField = Delaunay25DReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &cloud},
        parameters);
    require(heightField.succeeded, "CGAL Delaunay 2.5D should reconstruct a height field.");
    bool retainedHeight = false;
    for (const Point3d &vertex : heightField.mesh.vertices()) {
        retainedHeight = retainedHeight ||
            std::abs((vertex.vector() - projection.projection.basis.origin)
                         .dot(projection.projection.basis.normal)) > 1.0e-5;
    }
    require(retainedHeight, "Delaunay 2.5D should retain non-zero heights from the source cloud.");

    const PointCloud sphere = makeSphere(100U);
    const ReconstructionResult rejected = Delaunay25DReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &sphere},
        parameters);
    require(!rejected.succeeded, "Delaunay should reject a clearly volumetric cloud.");
}

void testCgalAlphaShapeAutomaticScale()
{
    PointCloud cloud = makePlane(7U, 0.25);
    for (std::size_t row = 0; row < 4U; ++row) {
        for (std::size_t column = 0; column < 4U; ++column) {
            cloud.addPoint(4.0 + static_cast<double>(column) * 0.25,
                           static_cast<double>(row) * 0.25,
                           0.0);
        }
    }

    AlphaShapeParameters parameters;
    parameters.automaticAlpha = true;
    parameters.automaticAlphaFactor = 4.0;
    parameters.planarityTolerance = 0.02;
    parameters.mode = PlanarReconstructionMode::Planar2D;
    const ReconstructionResult result = AlphaShapeReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &cloud},
        parameters);
    require(result.succeeded, "CGAL Alpha Shapes should reconstruct separated planar samples.");
    requireConsistentMesh(result.mesh);
    for (const Triangle &triangle : result.mesh.triangles()) {
        const auto &indices = triangle.vertexIndices;
        for (std::size_t edge = 0; edge < 3U; ++edge) {
            const double length =
                (result.mesh.vertices()[indices[edge]].vector() -
                 result.mesh.vertices()[indices[(edge + 1U) % 3U]].vector()).norm();
            require(length < 1.0, "Alpha Shapes should not bridge widely separated point groups.");
        }
    }
}
#endif

ReconstructionResult reconstruct(
    const PointCloud &cloud,
    const NormalField &normals,
    double ballRadius)
{
    const KDTree tree(cloud);
    ReconstructionParameters parameters;
    parameters.ballRadius = ballRadius;
    const BallPivotingReconstructor reconstructor;
    return reconstructor.reconstruct(cloud, normals, tree, parameters);
}

void testBallPivotingPlane()
{
    const PointCloud cloud = makePlane(6U, 0.5);
    const ReconstructionResult result = reconstruct(
        cloud,
        constantNormals(cloud.pointCount(), Eigen::Vector3d::UnitZ()),
        0.45);
    require(result.succeeded, "Ball Pivoting should reconstruct a regular planar sample.");
    require(result.mesh.triangleCount() >= 20U, "The planar reconstruction should produce a useful triangle set.");
    requireConsistentMesh(result.mesh);
}

void testBallPivotingCube()
{
    PointCloud cloud;
    for (double x : {-0.5, 0.5}) {
        for (double y : {-0.5, 0.5}) {
            for (double z : {-0.5, 0.5}) {
                cloud.addPoint(x, y, z);
            }
        }
    }
    const ReconstructionResult result = reconstruct(cloud, normalsFromPositions(cloud), 0.8);
    require(result.succeeded, "Ball Pivoting should reconstruct a synthetic cube vertex set.");
    require(result.mesh.triangleCount() >= 4U, "The cube reconstruction should produce multiple faces.");
    requireConsistentMesh(result.mesh);
}

void testBallPivotingSphere()
{
    const PointCloud cloud = makeSphere(120U);
    const ReconstructionResult result = reconstruct(cloud, normalsFromPositions(cloud), 0.36);
    require(result.succeeded, "Ball Pivoting should reconstruct a sampled sphere.");
    require(result.mesh.triangleCount() >= 60U, "The sphere reconstruction should contain a substantial triangle set.");
    requireConsistentMesh(result.mesh);
}

void testCompletePipeline()
{
    const PointCloud cloud = makePlane(6U, 0.5, true);
    ReconstructionPipelineParameters parameters;
    parameters.normalEstimation.neighbourCount = 10U;
    parameters.normalEstimation.maximumSearchRadius = 1.2;
    parameters.reconstruction.ballRadius = 0.45;

    const ReconstructionPipelineResult result = ReconstructionPipeline().execute(cloud, parameters);
    require(result.succeeded, "The PCA-to-Ball-Pivoting pipeline should complete on a noisy plane.");
    require(result.normalField.size() == cloud.pointCount(), "The pipeline should retain one normal entry per point.");
    require(
        !result.normalField.consistentlyOriented(),
        "Point-cloud estimators should not claim globally oriented normals.");
    requireConsistentMesh(result.mesh);
}

ScalarGrid3D makeScalarGrid(
    const Eigen::Vector3i &dimensions,
    const Eigen::Vector3d &origin,
    double spacing)
{
    ScalarGrid3D::CreateResult created = ScalarGrid3D::create(
        dimensions,
        origin,
        spacing,
        1024U * 1024U);
    require(created.grid.has_value(), "The scalar test grid should be created safely.");
    return std::move(*created.grid);
}

void testScalarGridSafety()
{
    const ScalarGrid3D::CreateResult tooLarge = ScalarGrid3D::create(
        Eigen::Vector3i(100, 100, 100),
        Eigen::Vector3d::Zero(),
        1.0,
        1000U);
    require(!tooLarge.grid.has_value(), "Scalar-grid creation should enforce the voxel limit.");
}

void testMarchingCubesPlane()
{
    ScalarGrid3D grid = makeScalarGrid(
        Eigen::Vector3i(8, 8, 8),
        Eigen::Vector3d(-1.0, -1.0, -1.0),
        0.3);
    for (int z = 0; z < grid.dimensions().z(); ++z) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int x = 0; x < grid.dimensions().x(); ++x) {
                grid.value(x, y, z) = static_cast<float>(grid.position(x, y, z).z());
            }
        }
    }
    const ReconstructionResult result = MarchingCubesReconstructor().extract(
        grid,
        MarchingCubesParameters{});
    require(result.succeeded, "Marching Cubes should extract a plane field.");
    requireConsistentMesh(result.mesh);
}

void testMarchingCubesSphereAndEmptyField()
{
    ScalarGrid3D grid = makeScalarGrid(
        Eigen::Vector3i(18, 18, 18),
        Eigen::Vector3d(-1.275, -1.275, -1.275),
        0.15);
    for (int z = 0; z < grid.dimensions().z(); ++z) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int x = 0; x < grid.dimensions().x(); ++x) {
                grid.value(x, y, z) = static_cast<float>(1.0 - grid.position(x, y, z).norm());
            }
        }
    }
    MarchingCubesParameters parameters;
    parameters.isoValue = 0.0;
    const ReconstructionResult sphere = MarchingCubesReconstructor().extract(grid, parameters);
    require(sphere.succeeded, "Marching Cubes should extract a sphere signed-distance field.");
    requireConsistentMesh(sphere.mesh);
    require(sphere.diagnostics.nonManifoldEdgeCount == 0U, "The sphere should have no non-manifold edges.");

    parameters.isoValue = 10.0;
    const ReconstructionResult empty = MarchingCubesReconstructor().extract(grid, parameters);
    require(empty.succeeded && empty.mesh.empty(), "An outside iso-value should produce an empty valid result.");
}

PointCloud makeCubeSurface(std::size_t samplesPerEdge)
{
    PointCloud cloud;
    for (std::size_t z = 0; z < samplesPerEdge; ++z) {
        for (std::size_t y = 0; y < samplesPerEdge; ++y) {
            for (std::size_t x = 0; x < samplesPerEdge; ++x) {
                if (x != 0U && y != 0U && z != 0U &&
                    x + 1U != samplesPerEdge && y + 1U != samplesPerEdge &&
                    z + 1U != samplesPerEdge) {
                    continue;
                }
                const auto coordinate = [samplesPerEdge](std::size_t index) {
                    return -1.0 + 2.0 * static_cast<double>(index) /
                        static_cast<double>(samplesPerEdge - 1U);
                };
                cloud.addPoint(coordinate(x), coordinate(y), coordinate(z));
            }
        }
    }
    return cloud;
}

void testVoxelReconstructionAndLimit()
{
    VoxelReconstructionParameters parameters;
    parameters.voxelSize = 0.25;
    parameters.maximumVoxelCount = 100000U;
    parameters.closeSmallGaps = true;

    const PointCloud cube = makeCubeSurface(9U);
    const ReconstructionResult cubeResult = VoxelReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &cube},
        parameters);
    require(cubeResult.succeeded, "Voxel reconstruction should extract a sampled cube.");
    requireConsistentMesh(cubeResult.mesh);

    const PointCloud sphere = makeSphere(300U);
    const ReconstructionResult sphereResult = VoxelReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &sphere},
        parameters);
    require(sphereResult.succeeded, "Voxel reconstruction should extract a sampled sphere.");
    requireConsistentMesh(sphereResult.mesh);

    parameters.maximumVoxelCount = 8U;
    const ReconstructionResult limited = VoxelReconstructor().reconstruct(
        ReconstructionInput{.pointCloud = &cube},
        parameters);
    require(!limited.succeeded, "Voxel reconstruction should reject an unsafe allocation.");
}

void testGreedyProjection()
{
    const PointCloud plane = makePlane(8U, 0.25);
    const NormalField planeNormals = constantNormals(plane.pointCount(), Eigen::Vector3d::UnitZ());
    const KDTree planeTree(plane);
    GreedyProjectionParameters parameters;
    parameters.searchRadius = 0.38;
    parameters.maximumNeighbours = 12U;
    const ReconstructionResult planeResult = GreedyProjectionReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &plane,
            .normalField = &planeNormals,
            .neighbourSearch = &planeTree,
        },
        parameters);
    require(planeResult.succeeded, "Greedy Projection should reconstruct a planar grid.");
    requireConsistentMesh(planeResult.mesh);

    const PointCloud sphere = makeSphere(180U);
    const NormalField sphereNormals = normalsFromPositions(sphere);
    const KDTree sphereTree(sphere);
    parameters.searchRadius = 0.42;
    parameters.maximumNeighbours = 16U;
    const ReconstructionResult sphereResult = GreedyProjectionReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &sphere,
            .normalField = &sphereNormals,
            .neighbourSearch = &sphereTree,
        },
        parameters);
    require(sphereResult.succeeded, "Greedy Projection should reconstruct a curved patch set.");
    requireConsistentMesh(sphereResult.mesh);

    const NormalField missingNormals;
    const ReconstructionResult rejected = GreedyProjectionReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &plane,
            .normalField = &missingNormals,
            .neighbourSearch = &planeTree,
        },
        parameters);
    require(!rejected.succeeded, "Greedy Projection should reject missing normals.");
}

void testRbfReconstructionAndLimits()
{
    const PointCloud sphere = makeSphere(72U);
    const NormalField normals = normalsFromPositions(sphere);
    RbfParameters parameters;
    parameters.kernel = RbfKernel::WendlandC2;
    parameters.supportRadius = 1.1;
    parameters.regularisation = 1.0e-6;
    parameters.maximumControlPoints = 32U;
    parameters.gridSpacing = 0.25;
    parameters.maximumVoxelCount = 100000U;
    parameters.maximumFieldEvaluations = 20000000U;
    const ReconstructionResult reconstructed = RbfReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &sphere,
            .normalField = &normals,
        },
        parameters);
    require(reconstructed.succeeded, "Bounded RBF should reconstruct a small oriented sphere.");
    requireConsistentMesh(reconstructed.mesh);

    const NormalField missingNormals;
    const ReconstructionResult missing = RbfReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &sphere,
            .normalField = &missingNormals,
        },
        parameters);
    require(!missing.succeeded, "RBF should reject missing oriented normals.");

    parameters.maximumControlPoints = 1024U;
    const ReconstructionResult excessiveControls = RbfReconstructor().reconstruct(
        ReconstructionInput{
            .pointCloud = &sphere,
            .normalField = &normals,
        },
        parameters);
    require(!excessiveControls.succeeded, "RBF should enforce its control-point safety limit.");
}

void testRegistryAndMultiMethodPipeline()
{
    const SurfaceReconstructorRegistry registry;
    require(registry.availability(ReconstructionMethod::BallPivoting).available, "BPA should be registered.");
    require(registry.availability(ReconstructionMethod::GreedyProjection).available, "Greedy Projection should be registered.");
    require(registry.availability(ReconstructionMethod::Voxel).available, "Voxel reconstruction should be registered.");
    require(registry.availability(ReconstructionMethod::Rbf).available, "The bounded RBF backend should be registered.");
    require(
        !registry.availability(ReconstructionMethod::ScreenedPoisson).available,
        "Screened Poisson should report its unavailable backend.");
#if NEWELL_HAS_CGAL
    require(registry.availability(ReconstructionMethod::Delaunay25D).available, "CGAL Delaunay should be available.");
#else
    require(!registry.availability(ReconstructionMethod::Delaunay25D).available, "Missing CGAL should disable Delaunay.");
    require(!registry.availability(ReconstructionMethod::AlphaShapes).available, "Missing CGAL should disable Alpha Shapes.");
#endif

    const PointCloud cube = makeCubeSurface(9U);
    ReconstructionPipelineParameters voxelParameters;
    voxelParameters.method = ReconstructionMethod::Voxel;
    VoxelReconstructionParameters voxel;
    voxel.voxelSize = 0.25;
    voxel.maximumVoxelCount = 100000U;
    voxelParameters.methodParameters = voxel;
    const ReconstructionPipelineResult voxelResult = ReconstructionPipeline().execute(cube, voxelParameters);
    require(voxelResult.succeeded, "The pipeline should dispatch voxel reconstruction.");
    requireConsistentMesh(voxelResult.mesh);
    require(voxelResult.normalField.empty(), "Voxel reconstruction should not estimate unused normals.");

    const PointCloud plane = makePlane(8U, 0.25);
    ReconstructionPipelineParameters greedyParameters;
    greedyParameters.method = ReconstructionMethod::GreedyProjection;
    greedyParameters.normalEstimation.neighbourCount = 12U;
    greedyParameters.normalEstimation.maximumSearchRadius = 0.5;
    GreedyProjectionParameters greedy;
    greedy.searchRadius = 0.38;
    greedy.maximumNeighbours = 12U;
    greedyParameters.methodParameters = greedy;
    const ReconstructionPipelineResult greedyResult = ReconstructionPipeline().execute(plane, greedyParameters);
    require(greedyResult.succeeded, "The pipeline should dispatch Greedy Projection.");
    requireConsistentMesh(greedyResult.mesh);

    ReconstructionPipelineParameters rbfParameters;
    rbfParameters.method = ReconstructionMethod::Rbf;
    rbfParameters.methodParameters = RbfParameters{};
    const ReconstructionPipelineResult unavailableRbf =
        ReconstructionPipeline().execute(plane, rbfParameters);
    require(
        !unavailableRbf.succeeded &&
            unavailableRbf.errorMessage.find("consistently oriented normals") != std::string::npos,
        "RBF should remain unavailable without an explicit normal-orientation stage.");
}

TriangleMesh makeSingleTriangleMesh(double z)
{
    TriangleMesh mesh;
    mesh.setVertices({Point3d(0.0, 0.0, z), Point3d(1.0, 0.0, z), Point3d(0.0, 1.0, z)});
    mesh.addTriangle(0U, 1U, 2U);
    return mesh;
}

void testReconstructedMeshDocumentState()
{
    GeometryDocument document;
    document.setTemporaryReconstructedMesh(makeSingleTriangleMesh(0.0));
    require(document.hasTemporaryReconstructedMesh(), "A converted mesh should begin as temporary state.");
    require(document.commitTemporaryReconstructedMesh(), "Apply should commit the temporary mesh.");
    require(!document.hasTemporaryReconstructedMesh(), "Applying should clear temporary mesh state.");
    require(document.committedReconstructedMesh() != nullptr, "Applying should retain a committed mesh.");

    document.setTemporaryReconstructedMesh(makeSingleTriangleMesh(1.0));
    require(
        document.displayedReconstructedMesh()->vertices().front().z() == 1.0,
        "A new temporary mesh should be displayed as the preview.");
    document.discardTemporaryReconstructedMesh();
    require(
        document.displayedReconstructedMesh()->vertices().front().z() == 0.0,
        "Cancel should restore the previously committed mesh.");
}
}

int main()
{
    try {
        testKDTreeQueries();
        testPcaPlaneNormals();
        testPcaNoisyPlaneNormals();
        testPcaSphereNormals();
        testPlanarProjectionAndPlanarityRejection();
#if NEWELL_HAS_CGAL
        testCgalDelaunayModesAndPlanarity();
        testCgalAlphaShapeAutomaticScale();
#endif
        testBallPivotingPlane();
        testBallPivotingCube();
        testBallPivotingSphere();
        testCompletePipeline();
        testScalarGridSafety();
        testMarchingCubesPlane();
        testMarchingCubesSphereAndEmptyField();
        testVoxelReconstructionAndLimit();
        testGreedyProjection();
        testRbfReconstructionAndLimits();
        testRegistryAndMultiMethodPipeline();
        testReconstructedMeshDocumentState();
    } catch (const std::exception &exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    return 0;
}
