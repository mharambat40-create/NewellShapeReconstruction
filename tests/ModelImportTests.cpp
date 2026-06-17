#include "model/geometry/PointCloud.h"
#include "model/io/PlyPointCloudImporter.h"

#include <filesystem>
#include <fstream>
#include <iostream>
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
}

int main()
{
    try {
        testPointCloudBasics();
        testAsciiPlyImport();
    } catch (const std::exception &exception) {
        std::cerr << "Test failure: " << exception.what() << '\n';
        return 1;
    }

    return 0;
}
