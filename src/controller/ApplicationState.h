#ifndef NEWELL_CONTROLLER_APPLICATIONSTATE_H
#define NEWELL_CONTROLLER_APPLICATIONSTATE_H

#include "model/geometry/GeometryDocument.h"
#include "model/io/PlyPointCloudImporter.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

struct LoadPointCloudResult
{
    bool success = false;
    std::string errorMessage;
    std::size_t pointCount = 0;
};

struct SparseSelectionResult
{
    bool success = false;
    std::string errorMessage;
    std::vector<std::size_t> selectedIndices;
};

struct RemovePointResult
{
    bool success = false;
    std::string errorMessage;
    std::size_t removedCount = 0;
    std::size_t remainingPointCount = 0;
};

class ApplicationState
{
public:
    [[nodiscard]] LoadPointCloudResult loadPointCloudFromFile(const std::filesystem::path &filePath);
    [[nodiscard]] SparseSelectionResult selectSparseCurrentPoints(
        double radiusMax,
        int minimumNeighbourCount) const;
    [[nodiscard]] RemovePointResult removeCurrentPointIndices(
        const std::vector<std::size_t> &selectedIndices);
    void restoreCurrentPointCloud(const PointCloud &pointCloud);
    [[nodiscard]] bool hasGeometryLoaded() const;
    [[nodiscard]] std::size_t currentPointCount() const;
    [[nodiscard]] std::size_t geometryRevision() const;
    [[nodiscard]] const PointCloud *currentPointCloud() const;
    [[nodiscard]] const GeometryDocument &document() const;

private:
    GeometryDocument document_;
    PlyPointCloudImporter importer_;
};

#endif // NEWELL_CONTROLLER_APPLICATIONSTATE_H
