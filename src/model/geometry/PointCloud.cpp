#include "model/geometry/PointCloud.h"

void PointCloud::addPoint(const Point3d &point)
{
    points_.push_back(point);
}

void PointCloud::addPoint(double x, double y, double z)
{
    points_.emplace_back(x, y, z);
}

void PointCloud::clear()
{
    points_.clear();
}

bool PointCloud::empty() const
{
    return points_.empty();
}

std::size_t PointCloud::pointCount() const
{
    return points_.size();
}

const PointCloud::Container &PointCloud::points() const
{
    return points_;
}

std::optional<BoundingBox3d> PointCloud::boundingBox() const
{
    if (points_.empty()) {
        return std::nullopt;
    }

    BoundingBox3d boundingBox;
    for (const Point3d &point : points_) {
        boundingBox.expandToInclude(point);
    }

    return boundingBox;
}
