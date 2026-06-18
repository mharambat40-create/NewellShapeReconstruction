#include "model/geometry/BoundingBox3d.h"

#include <algorithm>

void BoundingBox3d::expandToInclude(const Point3d &point)
{
    if (!valid_) {
        min_ = point.vector();
        max_ = point.vector();
        valid_ = true;
        return;
    }

    min_ = min_.cwiseMin(point.vector());
    max_ = max_.cwiseMax(point.vector());
}

bool BoundingBox3d::isValid() const
{
    return valid_;
}

Point3d BoundingBox3d::minPoint() const
{
    return Point3d(min_);
}

Point3d BoundingBox3d::maxPoint() const
{
    return Point3d(max_);
}

Eigen::Vector3d BoundingBox3d::center() const
{
    return (min_ + max_) * 0.5;
}

Eigen::Vector3d BoundingBox3d::extents() const
{
    return max_ - min_;
}

double BoundingBox3d::diagonalLength() const
{
    return extents().norm();
}

double BoundingBox3d::maxExtent() const
{
    const Eigen::Vector3d size = extents();
    return std::max({size.x(), size.y(), size.z()});
}
