#ifndef NEWELL_MODEL_PROCESSING_SPATIAL_KDTREE_H
#define NEWELL_MODEL_PROCESSING_SPATIAL_KDTREE_H

#include "model/processing/spatial/NeighbourSearch.h"

#include <cstddef>
#include <optional>
#include <vector>

class PointCloud;

// Balanced median-split index: O(n log n) construction, O(log n + k) average query cost.
class KDTree final : public INeighbourSearch
{
public:
    explicit KDTree(const PointCloud &pointCloud);

    [[nodiscard]] std::vector<Neighbour> kNearest(
        const Eigen::Vector3d &query,
        std::size_t neighbourCount,
        std::optional<std::size_t> excludedPoint = std::nullopt) const override;
    [[nodiscard]] std::vector<Neighbour> radiusSearch(
        const Eigen::Vector3d &query,
        double radius,
        std::optional<std::size_t> excludedPoint = std::nullopt) const override;

private:
    struct Node
    {
        std::size_t pointIndex = 0U;
        int left = -1;
        int right = -1;
        unsigned char axis = 0U;
    };

    int build(std::vector<std::size_t> &indices, std::size_t begin, std::size_t end, unsigned depth);
    void radiusSearchNode(
        int nodeIndex,
        const Eigen::Vector3d &query,
        double squaredRadius,
        std::optional<std::size_t> excludedPoint,
        std::vector<Neighbour> &result) const;

    const PointCloud &pointCloud_;
    std::vector<Node> nodes_;
    int root_ = -1;
};

#endif // NEWELL_MODEL_PROCESSING_SPATIAL_KDTREE_H
