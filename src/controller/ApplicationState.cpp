#include "controller/ApplicationState.h"

#include "model/preprocessing/DuplicatePointSelection.h"
#include "model/preprocessing/InvalidPointSelection.h"

#include <utility>

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

SparseSelectionResult ApplicationState::selectSparseCurrentPoints(
    double radiusMax,
    int minimumNeighbourCount) const
{
    const PointCloud *currentCloud = document_.currentPointCloud();
    if (!currentCloud) {
        return SparseSelectionResult{false, "No point cloud is loaded.", {}};
    }

    if (radiusMax <= 0.0) {
        return SparseSelectionResult{false, "Radius max must be greater than zero.", {}};
    }

    if (minimumNeighbourCount < 0) {
        return SparseSelectionResult{false, "Number of neighbours cannot be negative.", {}};
    }

    return SparseSelectionResult{
        true,
        {},
        selectSparseRegionPoints(*currentCloud, radiusMax, minimumNeighbourCount),
    };
}

SparseSelectionResult ApplicationState::selectPerfectDuplicateCurrentPoints() const
{
    const PointCloud *currentCloud = document_.currentPointCloud();
    if (!currentCloud) {
        return SparseSelectionResult{false, "No point cloud is loaded.", {}};
    }

    return SparseSelectionResult{
        true,
        {},
        selectPerfectDuplicatePoints(*currentCloud),
    };
}

SparseSelectionResult ApplicationState::selectNearDuplicateCurrentPoints(
    double distanceThreshold) const
{
    const PointCloud *currentCloud = document_.currentPointCloud();
    if (!currentCloud) {
        return SparseSelectionResult{false, "No point cloud is loaded.", {}};
    }

    if (distanceThreshold <= 0.0) {
        return SparseSelectionResult{false, "Distance threshold must be greater than zero.", {}};
    }

    return SparseSelectionResult{
        true,
        {},
        selectNearDuplicatePoints(*currentCloud, distanceThreshold),
    };
}

RemovePointResult ApplicationState::removeCurrentPointIndices(
    const std::vector<std::size_t> &selectedIndices)
{
    PointCloud *currentCloud = document_.currentPointCloud();
    if (!currentCloud) {
        return RemovePointResult{false, "No point cloud is loaded.", 0U, 0U};
    }

    if (selectedIndices.empty()) {
        return RemovePointResult{
            false,
            "No points are currently selected.",
            0U,
            currentCloud->pointCount(),
        };
    }

    const std::size_t previousCount = currentCloud->pointCount();
    PointCloud filteredCloud = removePointIndices(*currentCloud, selectedIndices);
    const std::size_t remainingCount = filteredCloud.pointCount();

    document_.replaceCurrentPointCloud(std::move(filteredCloud));

    return RemovePointResult{
        true,
        {},
        previousCount - remainingCount,
        remainingCount,
    };
}

void ApplicationState::restoreCurrentPointCloud(const PointCloud &pointCloud)
{
    document_.replaceCurrentPointCloud(pointCloud);
}

void ApplicationState::setTemporaryReconstructedMesh(TriangleMesh mesh)
{
    document_.setTemporaryReconstructedMesh(std::move(mesh));
}

bool ApplicationState::commitTemporaryReconstructedMesh()
{
    return document_.commitTemporaryReconstructedMesh();
}

void ApplicationState::discardTemporaryReconstructedMesh()
{
    document_.discardTemporaryReconstructedMesh();
}

bool ApplicationState::hasGeometryLoaded() const
{
    return document_.hasPointCloud();
}

std::size_t ApplicationState::currentPointCount() const
{
    return document_.currentPointCount();
}

std::size_t ApplicationState::geometryRevision() const
{
    return document_.revision();
}

const PointCloud *ApplicationState::currentPointCloud() const
{
    return document_.currentPointCloud();
}

bool ApplicationState::hasTemporaryReconstructedMesh() const
{
    return document_.hasTemporaryReconstructedMesh();
}

const TriangleMesh *ApplicationState::displayedReconstructedMesh() const
{
    return document_.displayedReconstructedMesh();
}

const GeometryDocument &ApplicationState::document() const
{
    return document_;
}
