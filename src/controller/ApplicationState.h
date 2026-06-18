#ifndef NEWELL_CONTROLLER_APPLICATIONSTATE_H
#define NEWELL_CONTROLLER_APPLICATIONSTATE_H

#include "model/geometry/GeometryDocument.h"
#include "model/io/PlyPointCloudImporter.h"

#include <cstddef>
#include <filesystem>
#include <string>

struct LoadPointCloudResult
{
    bool success = false;
    std::string errorMessage;
    std::size_t pointCount = 0;
};

class ApplicationState
{
public:
    [[nodiscard]] LoadPointCloudResult loadPointCloudFromFile(const std::filesystem::path &filePath);
    [[nodiscard]] bool hasGeometryLoaded() const;
    [[nodiscard]] std::size_t currentPointCount() const;
    [[nodiscard]] const PointCloud *currentPointCloud() const;
    [[nodiscard]] const GeometryDocument &document() const;

private:
    GeometryDocument document_;
    PlyPointCloudImporter importer_;
};

#endif // NEWELL_CONTROLLER_APPLICATIONSTATE_H
