#include "model/processing/normals/GraphNormalOrienter.h"

#include "model/geometry/PointCloud.h"
#include "model/processing/common/ProgressReporter.h"
#include "model/processing/normals/LocalSurfaceAnalysis.h"
#include "model/processing/spatial/NeighbourSearch.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <numeric>
#include <queue>
#include <set>
#include <vector>

namespace
{
struct GraphEdge
{
    std::size_t first = 0U;
    std::size_t second = 0U;
    double cost = 0.0;
};

class DisjointSet
{
public:
    explicit DisjointSet(std::size_t size)
        : parent_(size)
        , rank_(size, 0U)
    {
        std::iota(parent_.begin(), parent_.end(), 0U);
    }

    std::size_t find(std::size_t index)
    {
        if (parent_[index] != index) {
            parent_[index] = find(parent_[index]);
        }
        return parent_[index];
    }

    bool unite(std::size_t first, std::size_t second)
    {
        first = find(first);
        second = find(second);
        if (first == second) {
            return false;
        }
        if (rank_[first] < rank_[second]) {
            std::swap(first, second);
        }
        parent_[second] = first;
        if (rank_[first] == rank_[second]) {
            ++rank_[first];
        }
        return true;
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> rank_;
};
}

NormalMethod GraphNormalOrienter::method() const
{
    return NormalMethod::GraphOrientation;
}

NormalOrientationResult GraphNormalOrienter::orient(
    const PointCloud &pointCloud,
    const NormalField &inputNormals,
    const INeighbourSearch &neighbourSearch,
    const NormalMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    NormalOrientationResult result;
    ProgressReporter reporter(progressCallback);
    const auto *methodParameters = std::get_if<GraphOrientationParameters>(&parameters);
    if (!methodParameters) {
        result.errorMessage = "Graph orientation received parameters for another normal method.";
        return result;
    }
    if (inputNormals.size() != pointCloud.pointCount() || inputNormals.validNormalCount() == 0U) {
        result.errorMessage = "Graph orientation requires one normal entry per point and at least one valid normal.";
        return result;
    }
    if (!hasOnlyFiniteCoordinates(pointCloud)) {
        result.errorMessage = "Graph orientation requires finite point coordinates.";
        return result;
    }
    if (methodParameters->neighbourCount == 0U ||
        !std::isfinite(methodParameters->maximumPropagationAngleRadians) ||
        methodParameters->maximumPropagationAngleRadians < 0.0 ||
        methodParameters->maximumPropagationAngleRadians > std::numbers::pi_v<double> * 0.5 ||
        (methodParameters->seedOrientation == ComponentSeedOrientation::TowardViewpoint &&
         !methodParameters->viewpoint.allFinite())) {
        result.errorMessage = "Graph-orientation parameters are invalid.";
        return result;
    }

    reporter.report(ProcessingStage::BuildingNormalGraph, 0.0, "Building normal graph");
    std::vector<GraphEdge> edges;
    edges.reserve(pointCloud.pointCount() * methodParameters->neighbourCount / 2U);
    std::set<std::pair<std::size_t, std::size_t>> insertedEdges;
    for (std::size_t pointIndex = 0; pointIndex < pointCloud.pointCount(); ++pointIndex) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        if (!inputNormals.normal(pointIndex).isValid()) {
            continue;
        }
        const std::vector<Neighbour> neighbours = neighbourSearch.kNearest(
            pointCloud.points()[pointIndex].vector(),
            methodParameters->neighbourCount,
            pointIndex);
        for (const Neighbour &neighbour : neighbours) {
            if (!inputNormals.normal(neighbour.pointIndex).isValid()) {
                continue;
            }
            const auto edgeIndices = std::minmax(pointIndex, neighbour.pointIndex);
            if (!insertedEdges.insert(edgeIndices).second) {
                continue;
            }
            const double absoluteDot = std::clamp(
                std::abs(inputNormals.normal(pointIndex).vector().dot(
                    inputNormals.normal(neighbour.pointIndex).vector())),
                0.0,
                1.0);
            const double angle = std::acos(absoluteDot);
            if (angle <= methodParameters->maximumPropagationAngleRadians) {
                edges.push_back(GraphEdge{edgeIndices.first, edgeIndices.second, 1.0 - absoluteDot});
            }
        }
        if ((pointIndex & 127U) == 0U) {
            reporter.report(
                ProcessingStage::BuildingNormalGraph,
                0.55 * static_cast<double>(pointIndex) /
                    static_cast<double>(pointCloud.pointCount()),
                "Building normal graph");
        }
    }

    std::sort(edges.begin(), edges.end(), [](const GraphEdge &left, const GraphEdge &right) {
        if (left.cost != right.cost) {
            return left.cost < right.cost;
        }
        if (left.first != right.first) {
            return left.first < right.first;
        }
        return left.second < right.second;
    });

    DisjointSet forest(pointCloud.pointCount());
    std::vector<std::vector<std::size_t>> tree(pointCloud.pointCount());
    for (const GraphEdge &edge : edges) {
        if (forest.unite(edge.first, edge.second)) {
            tree[edge.first].push_back(edge.second);
            tree[edge.second].push_back(edge.first);
        }
    }
    for (auto &neighbours : tree) {
        std::sort(neighbours.begin(), neighbours.end());
    }

    Eigen::Vector3d cloudCentroid = Eigen::Vector3d::Zero();
    for (const Point3d &point : pointCloud.points()) {
        cloudCentroid += point.vector();
    }
    cloudCentroid /= static_cast<double>(pointCloud.pointCount());

    std::vector<Eigen::Vector3d> oriented(pointCloud.pointCount(), Eigen::Vector3d::Zero());
    std::vector<bool> visited(pointCloud.pointCount(), false);
    std::size_t processedValidNormals = 0U;
    reporter.report(ProcessingStage::OrientingNormals, 0.58, "Orienting normal components");
    for (std::size_t seed = 0; seed < pointCloud.pointCount(); ++seed) {
        if (visited[seed] || !inputNormals.normal(seed).isValid()) {
            continue;
        }
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }

        std::vector<std::size_t> component;
        std::queue<std::size_t> collect;
        collect.push(seed);
        visited[seed] = true;
        while (!collect.empty()) {
            if ((component.size() & 255U) == 0U &&
                cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            const std::size_t current = collect.front();
            collect.pop();
            component.push_back(current);
            for (const std::size_t neighbour : tree[current]) {
                if (!visited[neighbour]) {
                    visited[neighbour] = true;
                    collect.push(neighbour);
                }
            }
        }

        Eigen::Vector3d componentCentroid = Eigen::Vector3d::Zero();
        for (const std::size_t index : component) {
            componentCentroid += pointCloud.points()[index].vector();
        }
        componentCentroid /= static_cast<double>(component.size());

        oriented[seed] = inputNormals.normal(seed).vector();
        Eigen::Vector3d seedDirection = Eigen::Vector3d::Zero();
        if (methodParameters->seedOrientation == ComponentSeedOrientation::TowardViewpoint) {
            seedDirection = methodParameters->viewpoint - pointCloud.points()[seed].vector();
        } else if (methodParameters->seedOrientation ==
                   ComponentSeedOrientation::AwayFromCloudCentroid) {
            seedDirection = componentCentroid - cloudCentroid;
        }
        if (seedDirection.squaredNorm() > 1.0e-24 && oriented[seed].dot(seedDirection) < 0.0) {
            oriented[seed] = -oriented[seed];
        }

        std::set<std::size_t> propagated;
        std::queue<std::size_t> pending;
        propagated.insert(seed);
        pending.push(seed);
        while (!pending.empty()) {
            if ((propagated.size() & 255U) == 0U &&
                cancellationToken.isCancellationRequested()) {
                result.cancelled = true;
                result.errorMessage = "Processing cancelled.";
                reporter.cancelled(result.errorMessage);
                return result;
            }
            const std::size_t current = pending.front();
            pending.pop();
            for (const std::size_t neighbour : tree[current]) {
                if (!propagated.insert(neighbour).second) {
                    continue;
                }
                oriented[neighbour] = inputNormals.normal(neighbour).vector();
                if (oriented[current].dot(oriented[neighbour]) < 0.0) {
                    oriented[neighbour] = -oriented[neighbour];
                }
                pending.push(neighbour);
            }
        }

        ++result.diagnostics.connectedComponentCount;
        processedValidNormals += component.size();
        reporter.report(
            ProcessingStage::OrientingNormals,
            0.58 + 0.41 * static_cast<double>(processedValidNormals) /
                static_cast<double>(inputNormals.validNormalCount()),
            "Orienting normal components");
    }

    result.normalField.reserve(inputNormals.size());
    for (std::size_t index = 0; index < inputNormals.size(); ++index) {
        const Normal3d &normal = inputNormals.normal(index);
        if (normal.isValid()) {
            result.normalField.addNormal(oriented[index], normal.confidence());
        } else {
            result.normalField.addInvalidNormal();
        }
    }
    result.normalField.setConsistentlyOriented(
        processedValidNormals == inputNormals.validNormalCount());
    finalizeNormalDiagnostics(
        result.normalField,
        pointCloud.pointCount(),
        result.diagnostics);
    result.succeeded = result.normalField.consistentlyOriented();
    if (!result.succeeded) {
        result.errorMessage = "Graph orientation did not process every valid normal.";
        return result;
    }
    reporter.report(ProcessingStage::OrientingNormals, 0.99, "Normal components oriented");
    return result;
}
