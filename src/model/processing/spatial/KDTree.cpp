#include "model/processing/spatial/KDTree.h"

#include "model/geometry/PointCloud.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <utility>

KDTree::KDTree(const PointCloud &pointCloud)
    : pointCloud_(pointCloud)
{
    std::vector<std::size_t> indices(pointCloud.pointCount());
    std::iota(indices.begin(), indices.end(), 0U);
    nodes_.reserve(indices.size());
    root_ = build(indices, 0U, indices.size(), 0U);
}

std::vector<Neighbour> KDTree::kNearest(
    const Eigen::Vector3d &query,
    std::size_t neighbourCount,
    std::optional<std::size_t> excludedPoint) const
{
    if (neighbourCount == 0U || root_ < 0 || !query.allFinite()) {
        return {};
    }

    using HeapEntry = std::pair<double, std::size_t>;
    std::priority_queue<HeapEntry> nearest;

    const std::function<void(int)> visit = [&](int nodeIndex) {
        if (nodeIndex < 0) {
            return;
        }

        const Node &node = nodes_[static_cast<std::size_t>(nodeIndex)];
        const Eigen::Vector3d &point = pointCloud_.points()[node.pointIndex].vector();
        const double squaredDistance = (point - query).squaredNorm();
        if ((!excludedPoint || node.pointIndex != *excludedPoint) && std::isfinite(squaredDistance)) {
            if (nearest.size() < neighbourCount) {
                nearest.emplace(squaredDistance, node.pointIndex);
            } else if (squaredDistance < nearest.top().first ||
                       (squaredDistance == nearest.top().first && node.pointIndex < nearest.top().second)) {
                nearest.pop();
                nearest.emplace(squaredDistance, node.pointIndex);
            }
        }

        const double splitDistance = query[static_cast<int>(node.axis)] - point[static_cast<int>(node.axis)];
        const int nearChild = splitDistance < 0.0 ? node.left : node.right;
        const int farChild = splitDistance < 0.0 ? node.right : node.left;
        visit(nearChild);

        const double maximumAcceptedDistance = nearest.size() < neighbourCount
            ? std::numeric_limits<double>::infinity()
            : nearest.top().first;
        if (splitDistance * splitDistance <= maximumAcceptedDistance) {
            visit(farChild);
        }
    };

    visit(root_);

    std::vector<Neighbour> result;
    result.reserve(nearest.size());
    while (!nearest.empty()) {
        result.push_back(Neighbour{nearest.top().second, nearest.top().first});
        nearest.pop();
    }
    std::sort(result.begin(), result.end(), [](const Neighbour &left, const Neighbour &right) {
        return left.squaredDistance < right.squaredDistance ||
            (left.squaredDistance == right.squaredDistance && left.pointIndex < right.pointIndex);
    });
    return result;
}

std::vector<Neighbour> KDTree::radiusSearch(
    const Eigen::Vector3d &query,
    double radius,
    std::optional<std::size_t> excludedPoint) const
{
    if (radius < 0.0 || root_ < 0 || !query.allFinite()) {
        return {};
    }

    std::vector<Neighbour> result;
    radiusSearchNode(root_, query, radius * radius, excludedPoint, result);
    std::sort(result.begin(), result.end(), [](const Neighbour &left, const Neighbour &right) {
        return left.squaredDistance < right.squaredDistance ||
            (left.squaredDistance == right.squaredDistance && left.pointIndex < right.pointIndex);
    });
    return result;
}

int KDTree::build(
    std::vector<std::size_t> &indices,
    std::size_t begin,
    std::size_t end,
    unsigned depth)
{
    if (begin >= end) {
        return -1;
    }

    const unsigned char axis = static_cast<unsigned char>(depth % 3U);
    const std::size_t middle = begin + (end - begin) / 2U;
    std::nth_element(
        indices.begin() + static_cast<std::ptrdiff_t>(begin),
        indices.begin() + static_cast<std::ptrdiff_t>(middle),
        indices.begin() + static_cast<std::ptrdiff_t>(end),
        [this, axis](std::size_t left, std::size_t right) {
            const double leftCoordinate = pointCloud_.points()[left].vector()[axis];
            const double rightCoordinate = pointCloud_.points()[right].vector()[axis];
            return leftCoordinate < rightCoordinate ||
                (leftCoordinate == rightCoordinate && left < right);
        });

    const int nodeIndex = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{indices[middle], -1, -1, axis});
    const int left = build(indices, begin, middle, depth + 1U);
    const int right = build(indices, middle + 1U, end, depth + 1U);
    nodes_[static_cast<std::size_t>(nodeIndex)].left = left;
    nodes_[static_cast<std::size_t>(nodeIndex)].right = right;
    return nodeIndex;
}

void KDTree::radiusSearchNode(
    int nodeIndex,
    const Eigen::Vector3d &query,
    double squaredRadius,
    std::optional<std::size_t> excludedPoint,
    std::vector<Neighbour> &result) const
{
    if (nodeIndex < 0) {
        return;
    }

    const Node &node = nodes_[static_cast<std::size_t>(nodeIndex)];
    const Eigen::Vector3d &point = pointCloud_.points()[node.pointIndex].vector();
    const double squaredDistance = (point - query).squaredNorm();
    if ((!excludedPoint || node.pointIndex != *excludedPoint) && squaredDistance <= squaredRadius) {
        result.push_back(Neighbour{node.pointIndex, squaredDistance});
    }

    const double splitDistance = query[static_cast<int>(node.axis)] - point[static_cast<int>(node.axis)];
    const int nearChild = splitDistance < 0.0 ? node.left : node.right;
    const int farChild = splitDistance < 0.0 ? node.right : node.left;
    radiusSearchNode(nearChild, query, squaredRadius, excludedPoint, result);
    if (splitDistance * splitDistance <= squaredRadius) {
        radiusSearchNode(farChild, query, squaredRadius, excludedPoint, result);
    }
}
