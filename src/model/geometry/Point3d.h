#ifndef NEWELL_MODEL_GEOMETRY_POINT3D_H
#define NEWELL_MODEL_GEOMETRY_POINT3D_H

#include <Eigen/Core>

class Point3d
{
public:
    Point3d()
        : coordinates_(Eigen::Vector3d::Zero())
    {
    }

    Point3d(double x, double y, double z)
        : coordinates_(x, y, z)
    {
    }

    explicit Point3d(const Eigen::Vector3d &coordinates)
        : coordinates_(coordinates)
    {
    }

    [[nodiscard]] double x() const { return coordinates_.x(); }
    [[nodiscard]] double y() const { return coordinates_.y(); }
    [[nodiscard]] double z() const { return coordinates_.z(); }

    [[nodiscard]] const Eigen::Vector3d &vector() const { return coordinates_; }

private:
    Eigen::Vector3d coordinates_;
};

#endif // NEWELL_MODEL_GEOMETRY_POINT3D_H
