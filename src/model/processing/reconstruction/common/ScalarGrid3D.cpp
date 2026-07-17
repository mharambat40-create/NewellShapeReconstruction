#include "model/processing/reconstruction/common/ScalarGrid3D.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
std::optional<std::size_t> checkedProduct(const Eigen::Vector3i &dimensions)
{
    std::size_t product = 1U;
    for (int axis = 0; axis < 3; ++axis) {
        if (dimensions[axis] < 2) {
            return std::nullopt;
        }
        const std::size_t extent = static_cast<std::size_t>(dimensions[axis]);
        if (product > std::numeric_limits<std::size_t>::max() / extent) {
            return std::nullopt;
        }
        product *= extent;
    }
    return product;
}
}

ScalarGrid3D::CreateResult ScalarGrid3D::create(
    const Eigen::Vector3i &dimensions,
    const Eigen::Vector3d &origin,
    double spacing,
    std::size_t maximumVoxelCount,
    float initialValue)
{
    if (!origin.allFinite() || !std::isfinite(spacing) || spacing <= 0.0) {
        return {{}, "Scalar-grid origin and spacing must be finite, with positive spacing."};
    }
    if (!std::isfinite(initialValue)) {
        return {{}, "Scalar-grid initial value must be finite."};
    }
    const std::optional<std::size_t> count = checkedProduct(dimensions);
    if (!count) {
        return {{}, "Scalar-grid dimensions must be at least 2 in every axis and must not overflow."};
    }
    if (maximumVoxelCount == 0U || *count > maximumVoxelCount) {
        return {{}, "Scalar-grid allocation exceeds the configured voxel-count limit."};
    }
    if (*count > std::vector<float>().max_size()) {
        return {{}, "Scalar-grid allocation exceeds the container size limit."};
    }

    return {
        ScalarGrid3D(dimensions, origin, spacing, std::vector<float>(*count, initialValue)),
        {},
    };
}

ScalarGrid3D::ScalarGrid3D(
    Eigen::Vector3i dimensions,
    Eigen::Vector3d origin,
    double spacing,
    std::vector<float> values)
    : dimensions_(std::move(dimensions))
    , origin_(std::move(origin))
    , spacing_(spacing)
    , values_(std::move(values))
{
}

const Eigen::Vector3i &ScalarGrid3D::dimensions() const
{
    return dimensions_;
}

const Eigen::Vector3d &ScalarGrid3D::origin() const
{
    return origin_;
}

double ScalarGrid3D::spacing() const
{
    return spacing_;
}

std::size_t ScalarGrid3D::voxelCount() const
{
    return values_.size();
}

bool ScalarGrid3D::contains(int x, int y, int z) const
{
    return x >= 0 && y >= 0 && z >= 0 &&
        x < dimensions_.x() && y < dimensions_.y() && z < dimensions_.z();
}

std::optional<std::size_t> ScalarGrid3D::checkedIndex(int x, int y, int z) const
{
    if (!contains(x, y, z)) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(dimensions_.x()) *
            (static_cast<std::size_t>(y) +
             static_cast<std::size_t>(dimensions_.y()) * static_cast<std::size_t>(z));
}

float ScalarGrid3D::value(int x, int y, int z) const
{
    const std::optional<std::size_t> index = checkedIndex(x, y, z);
    if (!index) {
        throw std::out_of_range("Scalar-grid coordinate is outside the grid.");
    }
    return values_[*index];
}

float &ScalarGrid3D::value(int x, int y, int z)
{
    const std::optional<std::size_t> index = checkedIndex(x, y, z);
    if (!index) {
        throw std::out_of_range("Scalar-grid coordinate is outside the grid.");
    }
    return values_[*index];
}

Eigen::Vector3d ScalarGrid3D::position(int x, int y, int z) const
{
    return origin_ + spacing_ * Eigen::Vector3d(x, y, z);
}

const std::vector<float> &ScalarGrid3D::values() const
{
    return values_;
}

std::vector<float> &ScalarGrid3D::values()
{
    return values_;
}
