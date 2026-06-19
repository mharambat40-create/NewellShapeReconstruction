#include "model/geometry/GeometryDocument.h"

#include <utility>

void GeometryDocument::clear()
{
    originalPointCloud_.reset();
    currentPointCloud_.reset();
    bumpRevision();
}

void GeometryDocument::setPointCloud(const PointCloud &pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = pointCloud;
    bumpRevision();
}

void GeometryDocument::setPointCloud(PointCloud &&pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = std::move(pointCloud);
    bumpRevision();
}

void GeometryDocument::replaceCurrentPointCloud(const PointCloud &pointCloud)
{
    currentPointCloud_ = pointCloud;
    bumpRevision();
}

void GeometryDocument::replaceCurrentPointCloud(PointCloud &&pointCloud)
{
    currentPointCloud_ = std::move(pointCloud);
    bumpRevision();
}

bool GeometryDocument::hasPointCloud() const
{
    return originalPointCloud_.has_value() && currentPointCloud_.has_value();
}

std::size_t GeometryDocument::currentPointCount() const
{
    return currentPointCloud_ ? currentPointCloud_->pointCount() : 0U;
}

std::size_t GeometryDocument::revision() const
{
    return revision_;
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

void GeometryDocument::bumpRevision()
{
    ++revision_;
}
