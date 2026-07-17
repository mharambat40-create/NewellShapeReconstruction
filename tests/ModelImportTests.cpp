#include "model/geometry/BoundingBox3d.h"
#include "model/geometry/PointCloud.h"
#include "model/io/PlyPointCloudImporter.h"
#include "model/preprocessing/DuplicatePointSelection.h"
#include "model/preprocessing/InvalidPointSelection.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testPointCloudBasics()
{
    PointCloud pointCloud;
    require(pointCloud.empty(), "A new PointCloud should be empty.");

    pointCloud.addPoint(1.0, 2.0, 3.0);
    pointCloud.addPoint(Point3d(4.0, 5.0, 6.0));

    require(!pointCloud.empty(), "PointCloud should not be empty after adding points.");
    require(pointCloud.pointCount() == 2U, "PointCloud point count should be 2.");
    require(pointCloud.points().front().x() == 1.0, "First point x coordinate should match.");
}

void testAsciiPlyImport()
{
    const std::filesystem::path tempPath =
        std::filesystem::temp_directory_path() / "newell_ascii_import_test.ply";

    {
        std::ofstream output(tempPath);
        output << "ply\n";
        output << "format ascii 1.0\n";
        output << "element vertex 3\n";
        output << "property float x\n";
        output << "property float y\n";
        output << "property float z\n";
        output << "end_header\n";
        output << "0 0 0\n";
        output << "1 2 3\n";
        output << "4 5 6\n";
    }

    PlyPointCloudImporter importer;
    const PlyImportResult result = importer.importFromFile(tempPath);

    std::filesystem::remove(tempPath);

    require(result.success(), "ASCII PLY import should succeed.");
    require(result.pointCloud->pointCount() == 3U, "Imported point count should be 3.");
}

void testBoundingBox()
{
    PointCloud pointCloud;
    require(!pointCloud.boundingBox().has_value(), "Empty point clouds should not produce a bounding box.");

    pointCloud.addPoint(-1.0, 2.0, 0.5);
    pointCloud.addPoint(3.0, -4.0, 7.5);
    pointCloud.addPoint(2.0, 1.0, -2.0);

    const std::optional<BoundingBox3d> boundingBox = pointCloud.boundingBox();
    require(boundingBox.has_value(), "Bounding box should exist for non-empty clouds.");
    require(boundingBox->minPoint().x() == -1.0, "Bounding box min x should match.");
    require(boundingBox->minPoint().y() == -4.0, "Bounding box min y should match.");
    require(boundingBox->minPoint().z() == -2.0, "Bounding box min z should match.");
    require(boundingBox->maxPoint().x() == 3.0, "Bounding box max x should match.");
    require(boundingBox->maxPoint().y() == 2.0, "Bounding box max y should match.");
    require(boundingBox->maxPoint().z() == 7.5, "Bounding box max z should match.");
}

void testSparseSelection()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(0.3, 0.0, 0.0);
    pointCloud.addPoint(0.0, 0.3, 0.0);
    pointCloud.addPoint(10.0, 10.0, 10.0);

    const std::vector<std::size_t> selectedIndices =
        selectSparseRegionPoints(pointCloud, 0.5, 2);
    require(selectedIndices.size() == 1U, "Exactly one sparse point should be selected.");
    require(selectedIndices.front() == 3U, "The isolated point should be selected.");
}

void testSparseSelectionWithInvalidParameters()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);

    require(
        selectSparseRegionPoints(pointCloud, 0.0, 1).empty(),
        "Zero radius should be handled safely.");
    require(
        selectSparseRegionPoints(pointCloud, 1.0, -1).empty(),
        "Negative neighbour counts should be handled safely.");
    require(
        selectSparseRegionPoints(PointCloud{}, 1.0, 1).empty(),
        "Empty point clouds should produce an empty selection.");
}

void testPerfectDuplicateSelection()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(1.0, 1.0, 1.0);
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(1.0, 1.0, 1.0);
    pointCloud.addPoint(1.0, 1.0, 1.0);

    const std::vector<std::size_t> selectedIndices =
        selectPerfectDuplicatePoints(pointCloud);
    require(selectedIndices.size() == 3U, "Exact duplicate groups should keep one representative each.");
    require(selectedIndices[0] == 2U, "The second point in the first exact duplicate group should be selected.");
    require(selectedIndices[1] == 3U, "The second point in the second exact duplicate group should be selected.");
    require(selectedIndices[2] == 4U, "Subsequent points in an exact duplicate group should be selected.");
}

void testPerfectDuplicateSelectionWithNoDuplicates()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(1.0, 0.0, 0.0);
    pointCloud.addPoint(0.0, 1.0, 0.0);

    require(
        selectPerfectDuplicatePoints(pointCloud).empty(),
        "Point clouds without exact duplicates should produce an empty selection.");
}

void testNearDuplicateSelection()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(0.05, 0.0, 0.0);
    pointCloud.addPoint(1.0, 1.0, 1.0);
    pointCloud.addPoint(1.08, 1.0, 1.0);
    pointCloud.addPoint(3.0, 3.0, 3.0);

    const std::vector<std::size_t> selectedIndices =
        selectNearDuplicatePoints(pointCloud, 0.1);
    require(selectedIndices.size() == 2U, "Near duplicates within the threshold should select redundant points.");
    require(selectedIndices[0] == 1U, "The second point in the first near-duplicate group should be selected.");
    require(selectedIndices[1] == 3U, "The second point in the second near-duplicate group should be selected.");
}

void testNearDuplicateSelectionWithThresholdSeparation()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(0.2, 0.0, 0.0);
    pointCloud.addPoint(0.41, 0.0, 0.0);

    const std::vector<std::size_t> selectedIndices =
        selectNearDuplicatePoints(pointCloud, 0.1);
    require(
        selectedIndices.empty(),
        "Points farther than the threshold should not be selected as near duplicates.");
}

void testNearDuplicateSelectionWithInvalidParameters()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);

    require(
        selectNearDuplicatePoints(PointCloud{}, 0.1).empty(),
        "Empty point clouds should produce an empty near-duplicate selection.");
    require(
        selectNearDuplicatePoints(pointCloud, 0.0).empty(),
        "Zero distance threshold should be handled safely.");
    require(
        selectNearDuplicatePoints(pointCloud, -1.0).empty(),
        "Negative distance threshold should be handled safely.");
}

void testRemovePointIndices()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(1.0, 0.0, 0.0);
    pointCloud.addPoint(2.0, 0.0, 0.0);
    pointCloud.addPoint(3.0, 0.0, 0.0);

    const PointCloud filteredCloud = removePointIndices(pointCloud, {1U, 1U, 3U, 99U});
    require(filteredCloud.pointCount() == 2U, "Duplicate and out-of-range indices should be ignored safely.");
    require(filteredCloud.points()[0].x() == 0.0, "The first unselected point should be preserved.");
    require(filteredCloud.points()[1].x() == 2.0, "The second unselected point should be preserved.");
}

void testPointCloudSnapshotRestore()
{
    PointCloud pointCloud;
    pointCloud.addPoint(0.0, 0.0, 0.0);
    pointCloud.addPoint(1.0, 1.0, 1.0);
    pointCloud.addPoint(2.0, 2.0, 2.0);

    const PointCloud snapshot = pointCloud;
    pointCloud = removePointIndices(pointCloud, {1U});
    require(pointCloud.pointCount() == 2U, "Removing points from the working cloud should change the count.");

    pointCloud = snapshot;
    require(pointCloud.pointCount() == 3U, "Restoring the snapshot should recover the original count.");
    require(pointCloud.points()[1].x() == 1.0, "Restoring the snapshot should recover original coordinates.");
}
}

int main()
{
    try {
        testPointCloudBasics();
        testAsciiPlyImport();
        testBoundingBox();
        testSparseSelection();
        testSparseSelectionWithInvalidParameters();
        testPerfectDuplicateSelection();
        testPerfectDuplicateSelectionWithNoDuplicates();
        testNearDuplicateSelection();
        testNearDuplicateSelectionWithThresholdSeparation();
        testNearDuplicateSelectionWithInvalidParameters();
        testRemovePointIndices();
        testPointCloudSnapshotRestore();
    } catch (const std::exception &exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }

    return 0;
}
