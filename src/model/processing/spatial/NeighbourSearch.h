#ifndef NEWELL_MODEL_PROCESSING_SPATIAL_NEIGHBOURSEARCH_H
#define NEWELL_MODEL_PROCESSING_SPATIAL_NEIGHBOURSEARCH_H

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <vector>

struct Neighbour
{
    std::size_t pointIndex = 0U;
    double squaredDistance = 0.0;
};

class INeighbourSearch
{
public:
    virtual ~INeighbourSearch() = default;

    [[nodiscard]] virtual std::vector<Neighbour> kNearest(
        const Eigen::Vector3d &query,
        std::size_t neighbourCount,
        std::optional<std::size_t> excludedPoint = std::nullopt) const = 0;
    [[nodiscard]] virtual std::vector<Neighbour> radiusSearch(
        const Eigen::Vector3d &query,
        double radius,
        std::optional<std::size_t> excludedPoint = std::nullopt) const = 0;
};

#endif // NEWELL_MODEL_PROCESSING_SPATIAL_NEIGHBOURSEARCH_H
