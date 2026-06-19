#ifndef NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H
#define NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H

#include "model/geometry/PointCloud.h"

#include <cstddef>
#include <optional>

class GeometryDocument
{
public:
    void clear();
    void setPointCloud(const PointCloud &pointCloud);
    void setPointCloud(PointCloud &&pointCloud);
    void replaceCurrentPointCloud(const PointCloud &pointCloud);
    void replaceCurrentPointCloud(PointCloud &&pointCloud);

    [[nodiscard]] bool hasPointCloud() const;
    [[nodiscard]] std::size_t currentPointCount() const;
    [[nodiscard]] std::size_t revision() const;
    [[nodiscard]] const PointCloud *originalPointCloud() const;
    [[nodiscard]] const PointCloud *currentPointCloud() const;
    [[nodiscard]] PointCloud *currentPointCloud();

private:
    void bumpRevision();

    std::optional<PointCloud> originalPointCloud_;
    std::optional<PointCloud> currentPointCloud_;
    std::size_t revision_ = 0U;
};

#endif // NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H
