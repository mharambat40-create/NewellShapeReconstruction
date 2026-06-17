#include "controller/ApplicationState.h"

LoadPointCloudResult ApplicationState::loadPointCloudFromFile(const std::filesystem::path &filePath)
{
    const PlyImportResult importResult = importer_.importFromFile(filePath);
    if (!importResult.success()) {
        return LoadPointCloudResult{false, importResult.errorMessage, 0U};
    }

    document_.setPointCloud(*importResult.pointCloud);

    return LoadPointCloudResult{
        true,
        {},
        document_.currentPointCount(),
    };
}

bool ApplicationState::hasGeometryLoaded() const
{
    return document_.hasPointCloud();
}

std::size_t ApplicationState::currentPointCount() const
{
    return document_.currentPointCount();
}

const GeometryDocument &ApplicationState::document() const
{
    return document_;
}
