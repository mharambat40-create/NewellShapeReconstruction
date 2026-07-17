#include "model/geometry/GeometryDocument.h"

#include <utility>

void GeometryDocument::clear()
{
    originalPointCloud_.reset();
    currentPointCloud_.reset();
    clearReconstructedMeshes();
    bumpRevision();
}

void GeometryDocument::setPointCloud(const PointCloud &pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = pointCloud;
    clearReconstructedMeshes();
    bumpRevision();
}

void GeometryDocument::setPointCloud(PointCloud &&pointCloud)
{
    originalPointCloud_ = pointCloud;
    currentPointCloud_ = std::move(pointCloud);
    clearReconstructedMeshes();
    bumpRevision();
}

void GeometryDocument::replaceCurrentPointCloud(const PointCloud &pointCloud)
{
    currentPointCloud_ = pointCloud;
    clearReconstructedMeshes();
    bumpRevision();
}

void GeometryDocument::replaceCurrentPointCloud(PointCloud &&pointCloud)
{
    currentPointCloud_ = std::move(pointCloud);
    clearReconstructedMeshes();
    bumpRevision();
}

void GeometryDocument::setTemporaryReconstructedMesh(TriangleMesh mesh)
{
    temporaryReconstructedMesh_ = std::move(mesh);
}

bool GeometryDocument::commitTemporaryReconstructedMesh()
{
    if (!temporaryReconstructedMesh_) {
        return false;
    }

    committedReconstructedMesh_ = std::move(temporaryReconstructedMesh_);
    temporaryReconstructedMesh_.reset();
    return true;
}

void GeometryDocument::discardTemporaryReconstructedMesh()
{
    temporaryReconstructedMesh_.reset();
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

bool GeometryDocument::hasTemporaryReconstructedMesh() const
{
    return temporaryReconstructedMesh_.has_value();
}

const TriangleMesh *GeometryDocument::temporaryReconstructedMesh() const
{
    return temporaryReconstructedMesh_ ? &(*temporaryReconstructedMesh_) : nullptr;
}

const TriangleMesh *GeometryDocument::committedReconstructedMesh() const
{
    return committedReconstructedMesh_ ? &(*committedReconstructedMesh_) : nullptr;
}

const TriangleMesh *GeometryDocument::displayedReconstructedMesh() const
{
    if (temporaryReconstructedMesh_) {
        return &(*temporaryReconstructedMesh_);
    }
    return committedReconstructedMesh_ ? &(*committedReconstructedMesh_) : nullptr;
}

void GeometryDocument::bumpRevision()
{
    ++revision_;
}

void GeometryDocument::clearReconstructedMeshes()
{
    committedReconstructedMesh_.reset();
    temporaryReconstructedMesh_.reset();
}
