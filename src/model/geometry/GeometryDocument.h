#ifndef NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H
#define NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H

#include "model/geometry/PointCloud.h"
#include "model/geometry/TriangleMesh.h"

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
    void setTemporaryReconstructedMesh(TriangleMesh mesh);
    [[nodiscard]] bool commitTemporaryReconstructedMesh();
    void discardTemporaryReconstructedMesh();

    [[nodiscard]] bool hasPointCloud() const;
    [[nodiscard]] std::size_t currentPointCount() const;
    [[nodiscard]] std::size_t revision() const;
    [[nodiscard]] const PointCloud *originalPointCloud() const;
    [[nodiscard]] const PointCloud *currentPointCloud() const;
    [[nodiscard]] PointCloud *currentPointCloud();
    [[nodiscard]] bool hasTemporaryReconstructedMesh() const;
    [[nodiscard]] const TriangleMesh *temporaryReconstructedMesh() const;
    [[nodiscard]] const TriangleMesh *committedReconstructedMesh() const;
    [[nodiscard]] const TriangleMesh *displayedReconstructedMesh() const;

private:
    void bumpRevision();
    void clearReconstructedMeshes();

    std::optional<PointCloud> originalPointCloud_;
    std::optional<PointCloud> currentPointCloud_;
    std::optional<TriangleMesh> committedReconstructedMesh_;
    std::optional<TriangleMesh> temporaryReconstructedMesh_;
    std::size_t revision_ = 0U;
};

#endif // NEWELL_MODEL_GEOMETRY_GEOMETRYDOCUMENT_H
