#include "model/io/PlyPointCloudImporter.h"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::vector<std::string> splitWhitespace(const std::string &line)
{
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;

    while (stream >> token) {
        tokens.push_back(token);
    }

    return tokens;
}

PlyImportResult makeError(std::string message)
{
    return PlyImportResult{std::nullopt, std::move(message)};
}
}

PlyImportResult PlyPointCloudImporter::importFromFile(const std::filesystem::path &filePath) const
{
    std::ifstream input(filePath);
    if (!input) {
        return makeError("Failed to open file: " + filePath.string());
    }

    std::string line;
    if (!std::getline(input, line) || line != "ply") {
        return makeError("File is not a valid PLY file: missing 'ply' header.");
    }

    bool formatFound = false;
    bool isAscii = false;
    bool inVertexElement = false;
    bool headerComplete = false;
    std::size_t vertexCount = 0;
    std::size_t xIndex = 0;
    std::size_t yIndex = 1;
    std::size_t zIndex = 2;
    std::size_t propertyCountInVertexElement = 0;
    bool hasX = false;
    bool hasY = false;
    bool hasZ = false;

    while (std::getline(input, line)) {
        const auto tokens = splitWhitespace(line);
        if (tokens.empty()) {
            continue;
        }

        if (tokens[0] == "comment") {
            continue;
        }

        if (tokens[0] == "format") {
            if (tokens.size() < 2) {
                return makeError("Malformed PLY header: format line is incomplete.");
            }

            formatFound = true;
            isAscii = tokens[1] == "ascii";
            if (!isAscii) {
                return makeError("Only ASCII PLY files are supported. Binary PLY is rejected.");
            }
            continue;
        }

        if (tokens[0] == "element") {
            inVertexElement = false;
            propertyCountInVertexElement = 0;

            if (tokens.size() < 3) {
                return makeError("Malformed PLY header: element line is incomplete.");
            }

            if (tokens[1] == "vertex") {
                inVertexElement = true;

                try {
                    vertexCount = static_cast<std::size_t>(std::stoull(tokens[2]));
                } catch (...) {
                    return makeError("Malformed PLY header: invalid vertex count.");
                }
            }

            continue;
        }

        if (tokens[0] == "property") {
            if (inVertexElement) {
                if (tokens.size() < 3) {
                    return makeError("Malformed PLY header: vertex property line is incomplete.");
                }

                const std::string &propertyName = tokens.back();
                if (propertyName == "x") {
                    xIndex = propertyCountInVertexElement;
                    hasX = true;
                } else if (propertyName == "y") {
                    yIndex = propertyCountInVertexElement;
                    hasY = true;
                } else if (propertyName == "z") {
                    zIndex = propertyCountInVertexElement;
                    hasZ = true;
                }

                ++propertyCountInVertexElement;
            }

            continue;
        }

        if (tokens[0] == "end_header") {
            headerComplete = true;
            break;
        }
    }

    if (!formatFound) {
        return makeError("Malformed PLY header: missing format declaration.");
    }

    if (!headerComplete) {
        return makeError("Malformed PLY header: missing end_header.");
    }

    if (!hasX || !hasY || !hasZ) {
        return makeError("Malformed PLY header: vertex properties x, y, z are required.");
    }

    PointCloud pointCloud;
    for (std::size_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
        if (!std::getline(input, line)) {
            return makeError("Malformed PLY file: unexpected end of file while reading vertices.");
        }

        const auto tokens = splitWhitespace(line);
        const std::size_t requiredColumns = std::max({xIndex, yIndex, zIndex}) + 1U;
        if (tokens.size() < requiredColumns) {
            return makeError("Malformed PLY file: vertex line does not contain x, y, z coordinates.");
        }

        try {
            const double x = std::stod(tokens[xIndex]);
            const double y = std::stod(tokens[yIndex]);
            const double z = std::stod(tokens[zIndex]);
            pointCloud.addPoint(x, y, z);
        } catch (...) {
            return makeError("Malformed PLY file: failed to parse vertex coordinates.");
        }
    }

    return PlyImportResult{std::move(pointCloud), {}};
}
