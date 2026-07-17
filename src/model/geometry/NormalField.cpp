#include "model/geometry/NormalField.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kMinimumNormalLength = 1.0e-15;
}

Normal3d::Normal3d(
    const Eigen::Vector3d &direction,
    bool valid,
    double confidence)
{
    const double length = direction.norm();
    valid_ = valid && direction.allFinite() && length > kMinimumNormalLength &&
        std::isfinite(confidence);
    if (valid_) {
        direction_ = direction / length;
        confidence_ = std::clamp(confidence, 0.0, 1.0);
    }
}

double Normal3d::x() const
{
    return direction_.x();
}

double Normal3d::y() const
{
    return direction_.y();
}

double Normal3d::z() const
{
    return direction_.z();
}

const Eigen::Vector3d &Normal3d::vector() const
{
    return direction_;
}

bool Normal3d::isValid() const
{
    return valid_;
}

double Normal3d::confidence() const
{
    return confidence_;
}

void NormalField::reserve(std::size_t normalCapacity)
{
    normals_.reserve(normalCapacity);
}

void NormalField::addNormal(const Normal3d &normal)
{
    normals_.push_back(normal);
    if (normal.isValid()) {
        ++validNormalCount_;
    }
}

void NormalField::addNormal(const Eigen::Vector3d &direction, double confidence)
{
    addNormal(Normal3d(direction, true, confidence));
}

void NormalField::addInvalidNormal()
{
    normals_.emplace_back();
}

void NormalField::clear()
{
    normals_.clear();
    validNormalCount_ = 0U;
    consistentlyOriented_ = false;
}

void NormalField::setConsistentlyOriented(bool consistentlyOriented)
{
    consistentlyOriented_ = consistentlyOriented && validNormalCount_ > 0U;
}

bool NormalField::empty() const
{
    return normals_.empty();
}

std::size_t NormalField::size() const
{
    return normals_.size();
}

std::size_t NormalField::validNormalCount() const
{
    return validNormalCount_;
}

const Normal3d &NormalField::normal(std::size_t index) const
{
    return normals_.at(index);
}

const std::vector<Normal3d> &NormalField::normals() const
{
    return normals_;
}

bool NormalField::consistentlyOriented() const
{
    return consistentlyOriented_;
}
