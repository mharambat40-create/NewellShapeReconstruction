#ifndef NEWELL_MODEL_IO_PLYPOINTCLOUDIMPORTER_H
#define NEWELL_MODEL_IO_PLYPOINTCLOUDIMPORTER_H

#include "model/geometry/PointCloud.h"

#include <filesystem>
#include <optional>
#include <string>

struct PlyImportResult
{
    std::optional<PointCloud> pointCloud;
    std::string errorMessage;

    [[nodiscard]] bool success() const { return pointCloud.has_value(); }
};

class PlyPointCloudImporter
{
public:
    [[nodiscard]] PlyImportResult importFromFile(const std::filesystem::path &filePath) const;
};

#endif // NEWELL_MODEL_IO_PLYPOINTCLOUDIMPORTER_H
