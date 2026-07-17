#include "model/geometry/NormalField.h"
#include "model/geometry/PointCloud.h"
#include "model/processing/common/CancellationToken.h"
#include "model/processing/normals/GraphNormalOrienter.h"
#include "model/processing/normals/ImplicitFieldGradientNormalGenerator.h"
#include "model/processing/normals/MultiScalePcaNormalEstimator.h"
#include "model/processing/normals/NormalMethodRegistry.h"
#include "model/processing/normals/PcaKnnNormalEstimator.h"
#include "model/processing/normals/PcaRadiusNormalEstimator.h"
#include "model/processing/normals/QuadraticSurfaceNormalEstimator.h"
#include "model/processing/normals/ScalarGridGradientNormalGenerator.h"
#include "model/processing/normals/ViewpointNormalOrienter.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"
#include "model/processing/spatial/KDTree.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class ProgressCapture
{
public:
    ProgressCallback callback()
    {
        return [this](const ProcessingProgress &progress) {
            values_.push_back(progress.fraction);
        };
    }

    void requireWorkerProgress(const std::string &methodName) const
    {
        require(!values_.empty(), methodName + " should report progress.");
        require(
            std::is_sorted(values_.begin(), values_.end()),
            methodName + " progress should be monotonic.");
        require(
            std::all_of(values_.begin(), values_.end(), [](double value) {
                return std::isfinite(value) && value >= 0.0 && value <= 0.99;
            }),
            methodName + " worker progress should remain in [0, 0.99].");
    }

private:
    std::vector<double> values_;
};

PointCloud makePlane(
    std::size_t sideLength,
    double spacing,
    double slopeX = 0.0,
    double slopeY = 0.0)
{
    PointCloud cloud;
    cloud.reserve(sideLength * sideLength);
    const double half = static_cast<double>(sideLength - 1U) * spacing * 0.5;
    for (std::size_t row = 0; row < sideLength; ++row) {
        for (std::size_t column = 0; column < sideLength; ++column) {
            const double x = static_cast<double>(column) * spacing - half;
            const double y = static_cast<double>(row) * spacing - half;
            cloud.addPoint(x, y, slopeX * x + slopeY * y);
        }
    }
    return cloud;
}

PointCloud makeParaboloid(std::size_t sideLength, double spacing)
{
    PointCloud cloud;
    cloud.reserve(sideLength * sideLength);
    const double half = static_cast<double>(sideLength - 1U) * spacing * 0.5;
    for (std::size_t row = 0; row < sideLength; ++row) {
        for (std::size_t column = 0; column < sideLength; ++column) {
            const double x = static_cast<double>(column) * spacing - half;
            const double y = static_cast<double>(row) * spacing - half;
            cloud.addPoint(x, y, 0.2 * x * x + 0.1 * y * y);
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
        const double z = 1.0 - 2.0 * (static_cast<double>(index) + 0.5) /
            static_cast<double>(pointCount);
        const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double angle = goldenAngle * static_cast<double>(index);
        cloud.addPoint(radial * std::cos(angle), radial * std::sin(angle), z);
    }
    return cloud;
}

void requireWellFormed(
    const NormalEstimationResult &result,
    std::size_t expectedSize,
    const std::string &methodName)
{
    require(result.succeeded, methodName + " should succeed: " + result.errorMessage);
    require(result.normalField.size() == expectedSize, methodName + " should preserve result size.");
    require(
        result.diagnostics.requestedPointCount == expectedSize,
        methodName + " diagnostics should report the requested count.");
    require(
        result.diagnostics.validNormalCount + result.diagnostics.invalidNormalCount == expectedSize,
        methodName + " diagnostics should account for every result.");
    require(
        result.diagnostics.validNormalCount == result.normalField.validNormalCount(),
        methodName + " diagnostics should match the normal field.");
    for (const Normal3d &normal : result.normalField.normals()) {
        if (normal.isValid()) {
            require(normal.vector().allFinite(), methodName + " normals should be finite.");
            require(
                std::abs(normal.vector().norm() - 1.0) < 1.0e-9,
                methodName + " normals should have unit length.");
        }
    }
}

void requireWellFormed(
    const NormalOrientationResult &result,
    std::size_t expectedSize,
    const std::string &methodName)
{
    require(result.succeeded, methodName + " should succeed: " + result.errorMessage);
    require(result.normalField.size() == expectedSize, methodName + " should preserve result size.");
    require(
        result.normalField.consistentlyOriented(),
        methodName + " should mark the accepted output consistently oriented.");
    require(
        result.diagnostics.validNormalCount + result.diagnostics.invalidNormalCount == expectedSize,
        methodName + " diagnostics should account for every result.");
    for (const Normal3d &normal : result.normalField.normals()) {
        if (normal.isValid()) {
            require(normal.vector().allFinite(), methodName + " normals should be finite.");
            require(
                std::abs(normal.vector().norm() - 1.0) < 1.0e-9,
                methodName + " normals should preserve unit length.");
        }
    }
}

void testPcaFixedRadius()
{
    PointCloud cloud = makePlane(9U, 0.2, 0.25, -0.15);
    cloud.addPoint(0.0, 0.0, 0.0);
    cloud.addPoint(0.0, 0.0, 0.0);
    cloud.addPoint(20.0, 20.0, 20.0);
    const KDTree tree(cloud);
    PcaRadiusParameters parameters;
    parameters.searchRadius = 0.46;
    parameters.minimumNeighbours = 5U;
    parameters.maximumNeighbours = 32U;
    ProgressCapture progress;
    const NormalEstimationResult result = PcaRadiusNormalEstimator().estimate(
        cloud,
        tree,
        parameters,
        progress.callback());
    requireWellFormed(result, cloud.pointCount(), "PCA fixed radius");
    progress.requireWorkerProgress("PCA fixed radius");
    require(!result.normalField.normal(cloud.pointCount() - 1U).isValid(),
            "A point with insufficient radius neighbours should remain invalid.");
    const Eigen::Vector3d expected(-0.25, 0.15, 1.0);
    require(
        std::abs(result.normalField.normal(40U).vector().dot(expected.normalized())) > 0.995,
        "Fixed-radius PCA should recover a tilted plane normal up to sign.");

    PointCloud invalidCloud = cloud;
    invalidCloud.clear();
    invalidCloud.addPoint(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    for (std::size_t index = 1U; index < cloud.pointCount(); ++index) {
        invalidCloud.addPoint(cloud.points()[index]);
    }
    const NormalEstimationResult invalid = PcaRadiusNormalEstimator().estimate(
        invalidCloud,
        tree,
        parameters);
    require(!invalid.succeeded, "Fixed-radius PCA should reject non-finite coordinates.");
}

void testMultiScalePca()
{
    PointCloud plane = makePlane(11U, 0.13);
    for (std::size_t index = 0U; index < 20U; ++index) {
        const Point3d &point = plane.points()[index * 3U];
        plane.addPoint(point.x() + 0.002, point.y() - 0.002, point.z());
    }
    const KDTree planeTree(plane);
    MultiScalePcaParameters parameters;
    parameters.neighbourCounts = {6U, 12U, 24U, 40U};
    parameters.minimumValidScales = 2U;
    ProgressCapture progress;
    const NormalEstimationResult planeResult = MultiScalePcaNormalEstimator().estimate(
        plane,
        planeTree,
        parameters,
        progress.callback());
    requireWellFormed(planeResult, plane.pointCount(), "Multi-scale PCA");
    progress.requireWorkerProgress("Multi-scale PCA");
    require(
        std::count_if(
            planeResult.diagnostics.selectedNeighbourCounts.begin(),
            planeResult.diagnostics.selectedNeighbourCounts.end(),
            [](std::size_t count) { return count > 0U; }) ==
            static_cast<std::ptrdiff_t>(planeResult.normalField.validNormalCount()),
        "Multi-scale PCA should report one selected scale for each valid normal.");

    const PointCloud sphere = makeSphere(180U);
    const KDTree sphereTree(sphere);
    const NormalEstimationResult sphereResult = MultiScalePcaNormalEstimator().estimate(
        sphere,
        sphereTree,
        parameters);
    requireWellFormed(sphereResult, sphere.pointCount(), "Multi-scale PCA sphere");
    require(
        sphereResult.normalField.validNormalCount() > sphere.pointCount() * 4U / 5U,
        "Multi-scale PCA should remain stable on a curved, varying neighbourhood.");
    for (std::size_t index = 0U; index < sphere.pointCount(); ++index) {
        const Normal3d &normal = sphereResult.normalField.normal(index);
        if (normal.isValid()) {
            require(
                std::abs(normal.vector().dot(sphere.points()[index].vector())) > 0.8,
                "Multi-scale angular stability should be sign independent on a sphere.");
        }
    }

    parameters.neighbourCounts = {1U, 2U};
    const NormalEstimationResult invalid = MultiScalePcaNormalEstimator().estimate(
        sphere,
        sphereTree,
        parameters);
    require(!invalid.succeeded, "Multi-scale PCA should reject invalid neighbourhood scales.");
}

void testQuadraticSurfaceFitting()
{
    QuadraticFitParameters parameters;
    parameters.neighbourCount = 24U;
    parameters.regularisation = 1.0e-8;
    parameters.maximumConditionNumber = 1.0e9;

    const PointCloud plane = makePlane(11U, 0.16, -0.1, 0.2);
    const KDTree planeTree(plane);
    ProgressCapture progress;
    const NormalEstimationResult planeResult = QuadraticSurfaceNormalEstimator().estimate(
        plane,
        planeTree,
        parameters,
        progress.callback());
    requireWellFormed(planeResult, plane.pointCount(), "Quadratic fitting");
    progress.requireWorkerProgress("Quadratic fitting");
    require(
        std::abs(
            planeResult.normalField.normal(60U).vector().dot(
                Eigen::Vector3d(0.1, -0.2, 1.0).normalized())) > 0.995,
        "Quadratic fitting should recover the tilted plane normal up to sign.");

    const PointCloud paraboloid = makeParaboloid(13U, 0.12);
    const KDTree paraboloidTree(paraboloid);
    const NormalEstimationResult paraboloidResult = QuadraticSurfaceNormalEstimator().estimate(
        paraboloid,
        paraboloidTree,
        parameters);
    requireWellFormed(paraboloidResult, paraboloid.pointCount(), "Quadratic paraboloid fitting");
    require(
        std::abs(paraboloidResult.normalField.normal(84U).z()) > 0.999,
        "The paraboloid centre normal should be vertical up to sign.");

    const PointCloud sphere = makeSphere(160U);
    const KDTree sphereTree(sphere);
    const NormalEstimationResult sphereResult = QuadraticSurfaceNormalEstimator().estimate(
        sphere,
        sphereTree,
        parameters);
    requireWellFormed(sphereResult, sphere.pointCount(), "Quadratic sphere fitting");
    require(
        sphereResult.normalField.validNormalCount() > sphere.pointCount() * 3U / 4U,
        "Quadratic fitting should produce normals on a sphere patch set.");

    PointCloud line;
    for (int index = 0; index < 20; ++index) {
        line.addPoint(static_cast<double>(index), 0.0, 0.0);
    }
    const KDTree lineTree(line);
    const NormalEstimationResult rankDeficient = QuadraticSurfaceNormalEstimator().estimate(
        line,
        lineTree,
        parameters);
    require(!rankDeficient.succeeded, "Quadratic fitting should reject rank-deficient neighbourhoods.");

    parameters.maximumConditionNumber = 1.01;
    const NormalEstimationResult illConditioned = QuadraticSurfaceNormalEstimator().estimate(
        paraboloid,
        paraboloidTree,
        parameters);
    require(!illConditioned.succeeded, "Quadratic fitting should enforce its condition-number limit.");
}

NormalField alternatingPlaneNormals(std::size_t count)
{
    NormalField field;
    field.reserve(count);
    for (std::size_t index = 0U; index < count; ++index) {
        Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
        if (index % 2U != 0U) {
            direction = -direction;
        }
        field.addNormal(direction, 0.75);
    }
    return field;
}

void testGraphOrientation()
{
    const PointCloud plane = makePlane(8U, 0.2);
    const KDTree tree(plane);
    const NormalField input = alternatingPlaneNormals(plane.pointCount());
    GraphOrientationParameters parameters;
    parameters.neighbourCount = 8U;
    parameters.maximumPropagationAngleRadians = std::numbers::pi_v<double> * 0.5;
    parameters.seedOrientation = ComponentSeedOrientation::PreserveInput;
    ProgressCapture progress;
    const NormalOrientationResult first = GraphNormalOrienter().orient(
        plane,
        input,
        tree,
        parameters,
        progress.callback());
    requireWellFormed(first, plane.pointCount(), "Graph orientation");
    progress.requireWorkerProgress("Graph orientation");
    require(first.diagnostics.connectedComponentCount == 1U,
            "A regular planar grid should form one normal component.");
    for (std::size_t index = 1U; index < first.normalField.size(); ++index) {
        require(
            first.normalField.normal(0U).vector().dot(first.normalField.normal(index).vector()) > 0.999,
            "Alternating planar signs should become coherent.");
        require(
            std::abs(first.normalField.normal(index).confidence() - 0.75) < 1.0e-12,
            "Graph orientation should preserve confidence.");
    }
    const NormalOrientationResult second = GraphNormalOrienter().orient(
        plane,
        input,
        tree,
        parameters);
    for (std::size_t index = 0U; index < first.normalField.size(); ++index) {
        require(
            first.normalField.normal(index).vector() == second.normalField.normal(index).vector(),
            "Graph orientation should be deterministic.");
    }

    PointCloud separated;
    NormalField separatedNormals;
    for (double offset : {0.0, 100.0}) {
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 3; ++x) {
                separated.addPoint(offset + x * 0.1, y * 0.1, 0.0);
                Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
                if ((x + y) % 2 != 0) {
                    direction = -direction;
                }
                separatedNormals.addNormal(direction);
            }
        }
    }
    const KDTree separatedTree(separated);
    parameters.neighbourCount = 4U;
    const NormalOrientationResult components = GraphNormalOrienter().orient(
        separated,
        separatedNormals,
        separatedTree,
        parameters);
    requireWellFormed(components, separated.pointCount(), "Disconnected graph orientation");
    require(components.diagnostics.connectedComponentCount == 2U,
            "Separated point sets should produce two connected components.");

    NormalField perpendicular;
    for (std::size_t index = 0U; index < plane.pointCount(); ++index) {
        perpendicular.addNormal(index % 2U == 0U
                                    ? Eigen::Vector3d::UnitZ()
                                    : Eigen::Vector3d::UnitX());
    }
    parameters.maximumPropagationAngleRadians = 0.1;
    const NormalOrientationResult angleLimited = GraphNormalOrienter().orient(
        plane,
        perpendicular,
        tree,
        parameters);
    requireWellFormed(angleLimited, plane.pointCount(), "Angle-limited graph orientation");
    require(angleLimited.diagnostics.connectedComponentCount > 1U,
            "The maximum propagation angle should disconnect incompatible normals.");
}

void testViewpointOrientation()
{
    PointCloud cloud = makeSphere(80U);
    cloud.addPoint(0.0, 0.0, 0.0);
    cloud.addPoint(2.0, 0.0, 0.0);
    NormalField input;
    for (std::size_t index = 0U; index < 80U; ++index) {
        const Eigen::Vector3d radial = cloud.points()[index].vector();
        input.addNormal(index % 2U == 0U ? radial : -radial, 0.6);
    }
    input.addNormal(Eigen::Vector3d::UnitX(), 0.4);
    input.addInvalidNormal();
    const KDTree tree(cloud);

    ViewpointOrientationParameters parameters;
    parameters.viewpoint = Eigen::Vector3d::Zero();
    parameters.pointTowardViewpoint = true;
    ProgressCapture progress;
    const NormalOrientationResult toward = ViewpointNormalOrienter().orient(
        cloud,
        input,
        tree,
        parameters,
        progress.callback());
    requireWellFormed(toward, cloud.pointCount(), "Viewpoint orientation");
    progress.requireWorkerProgress("Viewpoint orientation");
    for (std::size_t index = 0U; index < 80U; ++index) {
        require(
            toward.normalField.normal(index).vector().dot(-cloud.points()[index].vector()) > 0.0,
            "Toward-viewpoint normals should face the viewpoint.");
    }
    require(
        toward.normalField.normal(80U).vector() == Eigen::Vector3d::UnitX(),
        "A point coincident with the viewpoint should preserve its normal.");
    require(!toward.normalField.normal(81U).isValid(),
            "Viewpoint orientation should preserve invalid normals.");

    parameters.pointTowardViewpoint = false;
    const NormalOrientationResult away = ViewpointNormalOrienter().orient(
        cloud,
        input,
        tree,
        parameters);
    requireWellFormed(away, cloud.pointCount(), "Away-from-viewpoint orientation");
    for (std::size_t index = 0U; index < 80U; ++index) {
        require(
            away.normalField.normal(index).vector().dot(-cloud.points()[index].vector()) < 0.0,
            "Away-from-viewpoint normals should face away from the viewpoint.");
    }

    parameters.viewpoint.x() = std::numeric_limits<double>::infinity();
    require(
        !ViewpointNormalOrienter().orient(cloud, input, tree, parameters).succeeded,
        "Viewpoint orientation should reject a non-finite viewpoint.");
}

class SphereField final : public IImplicitField
{
public:
    double value(const Eigen::Vector3d &position) const override
    {
        return position.squaredNorm() - 1.0;
    }

    std::optional<Eigen::Vector3d> gradient(
        const Eigen::Vector3d &position) const override
    {
        return 2.0 * position;
    }
};

class PlaneField final : public IImplicitField
{
public:
    double value(const Eigen::Vector3d &position) const override
    {
        return 2.0 * position.x() - 3.0 * position.y() + 4.0 * position.z();
    }
};

class ConstantField final : public IImplicitField
{
public:
    double value(const Eigen::Vector3d &) const override
    {
        return 1.0;
    }
};

void testImplicitFieldGradients()
{
    PointCloud spherePoints = makeSphere(60U);
    spherePoints.addPoint(0.0, 0.0, 0.0);
    const SphereField sphereField;
    ImplicitGradientParameters parameters;
    ProgressCapture progress;
    const NormalEstimationResult analytical = ImplicitFieldGradientNormalGenerator().generate(
        FieldNormalInput{.queryPoints = &spherePoints, .implicitField = &sphereField},
        parameters,
        progress.callback());
    requireWellFormed(analytical, spherePoints.pointCount(), "Analytical implicit gradients");
    progress.requireWorkerProgress("Analytical implicit gradients");
    require(!analytical.normalField.normal(60U).isValid(),
            "A zero analytical gradient should be marked invalid.");
    for (std::size_t index = 0U; index < 60U; ++index) {
        require(
            analytical.normalField.normal(index).vector().dot(
                spherePoints.points()[index].vector()) > 0.999,
            "Sphere analytical gradients should point radially outward.");
    }

    PointCloud planeQueries;
    planeQueries.addPoint(0.0, 0.0, 0.0);
    planeQueries.addPoint(0.4, -0.2, 0.8);
    const PlaneField planeField;
    parameters.finiteDifferenceStep = 1.0e-4;
    const NormalEstimationResult finiteDifference =
        ImplicitFieldGradientNormalGenerator().generate(
            FieldNormalInput{.queryPoints = &planeQueries, .implicitField = &planeField},
            parameters);
    requireWellFormed(finiteDifference, planeQueries.pointCount(), "Finite-difference gradients");
    const Eigen::Vector3d expected = Eigen::Vector3d(2.0, -3.0, 4.0).normalized();
    require(
        finiteDifference.normalField.normal(0U).vector().dot(expected) > 0.999999,
        "Central finite differences should recover a linear field gradient.");

    const ConstantField constantField;
    const NormalEstimationResult zero = ImplicitFieldGradientNormalGenerator().generate(
        FieldNormalInput{.queryPoints = &planeQueries, .implicitField = &constantField},
        parameters);
    require(!zero.succeeded && zero.diagnostics.nearZeroGradientCount == planeQueries.pointCount(),
            "A constant implicit field should produce only invalid normals.");
}

ScalarGrid3D makeLinearGrid()
{
    ScalarGrid3D::CreateResult created = ScalarGrid3D::create(
        Eigen::Vector3i(6, 5, 4),
        Eigen::Vector3d(-1.0, -0.75, 0.2),
        0.35,
        1000U);
    require(created.grid.has_value(), "The scalar test grid should be allocated.");
    ScalarGrid3D grid = std::move(*created.grid);
    for (int z = 0; z < grid.dimensions().z(); ++z) {
        for (int y = 0; y < grid.dimensions().y(); ++y) {
            for (int x = 0; x < grid.dimensions().x(); ++x) {
                const Eigen::Vector3d position = grid.position(x, y, z);
                grid.value(x, y, z) = static_cast<float>(
                    2.0 * position.x() - 3.0 * position.y() + 4.0 * position.z());
            }
        }
    }
    return grid;
}

void testScalarGridGradients()
{
    ScalarGrid3D grid = makeLinearGrid();
    ScalarGridGradientParameters parameters;
    parameters.interpolateGradient = true;
    ProgressCapture progress;
    const NormalEstimationResult allSamples = ScalarGridGradientNormalGenerator().generate(
        FieldNormalInput{.scalarGrid = &grid},
        parameters,
        progress.callback());
    requireWellFormed(allSamples, grid.voxelCount(), "Scalar-grid gradients");
    progress.requireWorkerProgress("Scalar-grid gradients");
    const Eigen::Vector3d expected = Eigen::Vector3d(2.0, -3.0, 4.0).normalized();
    for (const Normal3d &normal : allSamples.normalField.normals()) {
        require(
            normal.vector().dot(expected) > 0.999999,
            "Central and one-sided differences should account for non-unit grid spacing.");
    }

    PointCloud queries;
    const Eigen::Vector3d boundary = grid.position(0, 0, 0);
    queries.addPoint(boundary.x(), boundary.y(), boundary.z());
    const Eigen::Vector3d interior =
        grid.origin() + grid.spacing() * Eigen::Vector3d(2.3, 1.4, 1.6);
    queries.addPoint(interior.x(), interior.y(), interior.z());
    const NormalEstimationResult interpolated = ScalarGridGradientNormalGenerator().generate(
        FieldNormalInput{.queryPoints = &queries, .scalarGrid = &grid},
        parameters);
    requireWellFormed(interpolated, queries.pointCount(), "Interpolated scalar-grid gradients");
    require(interpolated.normalField.normal(1U).vector().dot(expected) > 0.999999,
            "Trilinear interpolation should preserve a linear field gradient.");

    ScalarGrid3D::CreateResult sphereCreated = ScalarGrid3D::create(
        Eigen::Vector3i(7, 7, 7),
        Eigen::Vector3d(-1.5, -1.5, -1.5),
        0.5,
        1000U);
    require(sphereCreated.grid.has_value(), "The sphere-distance grid should be allocated.");
    ScalarGrid3D sphereGrid = std::move(*sphereCreated.grid);
    for (int z = 0; z < sphereGrid.dimensions().z(); ++z) {
        for (int y = 0; y < sphereGrid.dimensions().y(); ++y) {
            for (int x = 0; x < sphereGrid.dimensions().x(); ++x) {
                sphereGrid.value(x, y, z) =
                    static_cast<float>(sphereGrid.position(x, y, z).norm() - 1.0);
            }
        }
    }
    PointCloud sphereQueries;
    sphereQueries.addPoint(1.0, 0.0, 0.0);
    sphereQueries.addPoint(0.0, -1.0, 0.0);
    const NormalEstimationResult sphere = ScalarGridGradientNormalGenerator().generate(
        FieldNormalInput{.queryPoints = &sphereQueries, .scalarGrid = &sphereGrid},
        parameters);
    requireWellFormed(sphere, sphereQueries.pointCount(), "Sphere scalar-grid gradients");
    require(sphere.normalField.normal(0U).x() > 0.99 &&
                sphere.normalField.normal(1U).y() < -0.99,
            "Sphere-distance grid normals should follow the sampled radial gradient.");
}

void testRegistryRequirements()
{
    const NormalMethodRegistry registry;
    const auto exposedMethods = NormalMethodRegistry::pointCloudEstimatorMethods();
    require(exposedMethods.size() == 4U,
            "The current normal-selection workflow should expose exactly four estimators.");
    require(
        !NormalMethodRegistry::methodFromDisplayName(
             "Graph-based normal orientation propagation").has_value() &&
            !NormalMethodRegistry::methodFromDisplayName(
                 "Viewpoint-based normal orientation").has_value() &&
            !NormalMethodRegistry::methodFromDisplayName(
                 "Normals from implicit field gradient").has_value() &&
            !NormalMethodRegistry::methodFromDisplayName(
                 "Normals from voxel/scalar-field gradient").has_value(),
        "Future orientation and field methods should not be exposed by display-name mapping.");
    const NormalProcessingContext pointsOnly{
        .hasPointCloud = true,
        .hasValidNormals = false,
        .hasImplicitField = false,
        .hasScalarGrid = false,
    };
    for (const NormalMethod method : {
             NormalMethod::PcaKNearest,
             NormalMethod::PcaFixedRadius,
             NormalMethod::PcaMultiScale,
             NormalMethod::QuadraticSurfaceFit}) {
        require(registry.availability(method, pointsOnly).available,
                "Point estimators should be available with a point cloud.");
        require(registry.createEstimator(method) != nullptr,
                "Every point estimator should have a registry factory.");
    }
    require(!registry.availability(NormalMethod::GraphOrientation, pointsOnly).available,
            "Graph orientation should require existing normals.");
    require(!registry.availability(NormalMethod::ViewpointOrientation, pointsOnly).available,
            "Viewpoint orientation should require existing normals.");
    require(!registry.availability(NormalMethod::ImplicitFieldGradient, pointsOnly).available,
            "Implicit gradients should require an implicit field.");
    require(!registry.availability(NormalMethod::ScalarGridGradient, pointsOnly).available,
            "Scalar-grid gradients should require a scalar grid.");

    NormalProcessingContext complete = pointsOnly;
    complete.hasValidNormals = true;
    complete.hasImplicitField = true;
    complete.hasScalarGrid = true;
    require(registry.availability(NormalMethod::GraphOrientation, complete).available &&
                registry.createOrienter(NormalMethod::GraphOrientation) != nullptr,
            "Graph orientation should be available with valid normals.");
    require(registry.availability(NormalMethod::ViewpointOrientation, complete).available &&
                registry.createOrienter(NormalMethod::ViewpointOrientation) != nullptr,
            "Viewpoint orientation should be available with valid normals.");
    require(registry.availability(NormalMethod::ImplicitFieldGradient, complete).available &&
                registry.createFieldGenerator(NormalMethod::ImplicitFieldGradient) != nullptr,
            "Implicit gradients should be available with an implicit field.");
    require(registry.availability(NormalMethod::ScalarGridGradient, complete).available &&
                registry.createFieldGenerator(NormalMethod::ScalarGridGradient) != nullptr,
            "Scalar-grid gradients should be available with a scalar grid.");
}

void testCancellationForEveryMethod()
{
    const PointCloud plane = makePlane(8U, 0.2);
    const KDTree tree(plane);
    const NormalField normals = alternatingPlaneNormals(plane.pointCount());
    CancellationSource cancellation;
    cancellation.requestCancellation();
    const CancellationToken token = cancellation.token();

    require(PcaKnnNormalEstimator().estimate(
                plane, tree, PcaKnnParameters{}, {}, token).cancelled,
            "PCA kNN cancellation should be safe.");
    PcaRadiusParameters radius;
    radius.searchRadius = 0.5;
    require(PcaRadiusNormalEstimator().estimate(
                plane, tree, radius, {}, token).cancelled,
            "PCA fixed-radius cancellation should be safe.");
    require(MultiScalePcaNormalEstimator().estimate(
                plane, tree, MultiScalePcaParameters{}, {}, token).cancelled,
            "Multi-scale PCA cancellation should be safe.");
    require(QuadraticSurfaceNormalEstimator().estimate(
                plane, tree, QuadraticFitParameters{}, {}, token).cancelled,
            "Quadratic fitting cancellation should be safe.");
    require(GraphNormalOrienter().orient(
                plane, normals, tree, GraphOrientationParameters{}, {}, token).cancelled,
            "Graph orientation cancellation should be safe.");
    require(ViewpointNormalOrienter().orient(
                plane, normals, tree, ViewpointOrientationParameters{}, {}, token).cancelled,
            "Viewpoint orientation cancellation should be safe.");

    const SphereField implicitField;
    require(ImplicitFieldGradientNormalGenerator().generate(
                FieldNormalInput{.queryPoints = &plane, .implicitField = &implicitField},
                ImplicitGradientParameters{},
                {},
                token).cancelled,
            "Implicit-gradient cancellation should be safe.");
    ScalarGrid3D grid = makeLinearGrid();
    require(ScalarGridGradientNormalGenerator().generate(
                FieldNormalInput{.queryPoints = &plane, .scalarGrid = &grid},
                ScalarGridGradientParameters{},
                {},
                token).cancelled,
            "Scalar-grid-gradient cancellation should be safe.");
}
}

int main()
{
    try {
        testPcaFixedRadius();
        testMultiScalePca();
        testQuadraticSurfaceFitting();
        testGraphOrientation();
        testViewpointOrientation();
        testImplicitFieldGradients();
        testScalarGridGradients();
        testRegistryRequirements();
        testCancellationForEveryMethod();
    } catch (const std::exception &exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }
    return 0;
}
