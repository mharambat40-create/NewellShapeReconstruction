#ifndef NEWELL_MODEL_GEOMETRY_BOUNDINGBOX3D_H
#define NEWELL_MODEL_GEOMETRY_BOUNDINGBOX3D_H

#include "model/geometry/Point3d.h"

#include <Eigen/Core>

class BoundingBox3d
{
public:
    BoundingBox3d() = default;

    void expandToInclude(const Point3d &point);

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] Point3d minPoint() const;
    [[nodiscard]] Point3d maxPoint() const;
    [[nodiscard]] Eigen::Vector3d center() const;
    [[nodiscard]] Eigen::Vector3d extents() const;
    [[nodiscard]] double diagonalLength() const;
    [[nodiscard]] double maxExtent() const;

private:
    bool valid_ = false;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
};

#endif // NEWELL_MODEL_GEOMETRY_BOUNDINGBOX3D_H
