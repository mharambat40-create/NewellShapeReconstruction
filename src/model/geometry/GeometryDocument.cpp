#include "model/geometry/GeometryDocument.h"

#include <utility>

void GeometryDocument::clear()
{
    originalPointCloud_.reset();
    currentPointCloud_.reset();
}

void GeometryDocument::setPointCloud(const PointCloud &pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = pointCloud;
}

void GeometryDocument::setPointCloud(PointCloud &&pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = std::move(pointCloud);
}

bool GeometryDocument::hasPointCloud() const
{
    return originalPointCloud_.has_value() && currentPointCloud_.has_value();
}

std::size_t GeometryDocument::currentPointCount() const
{
    return currentPointCloud_ ? currentPointCloud_->pointCount() : 0U;
}

const PointCloud *GeometryDocument::originalPointCloud() const
{
    return originalPointCloud_ ? &(*originalPointCloud_) : nullptr;
}

const PointCloud *GeometryDocument::currentPointCloud() const
{
    return currentPointCloud_ ? &(*currentPointCloud_) : nullptr;
}

PointCloud *GeometryDocument::currentPointCloud()
{
    return currentPointCloud_ ? &(*currentPointCloud_) : nullptr;
}
