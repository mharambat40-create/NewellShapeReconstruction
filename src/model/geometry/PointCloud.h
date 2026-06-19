#ifndef NEWELL_MODEL_GEOMETRY_POINTCLOUD_H
#define NEWELL_MODEL_GEOMETRY_POINTCLOUD_H

#include "model/geometry/BoundingBox3d.h"
#include "model/geometry/Point3d.h"

#include <cstddef>
#include <optional>
#include <vector>

class PointCloud
{
public:
    using Container = std::vector<Point3d>;

    void addPoint(const Point3d &point);
    void addPoint(double x, double y, double z);
    void clear();
    void reserve(std::size_t pointCapacity);

    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::size_t pointCount() const;
    [[nodiscard]] const Container &points() const;
    [[nodiscard]] std::optional<BoundingBox3d> boundingBox() const;

private:
    Container points_;
};

#endif // NEWELL_MODEL_GEOMETRY_POINTCLOUD_H
