#include "model/processing/reconstruction/MarchingCubesReconstructor.h"

#include "model/processing/common/ProgressReporter.h"
#include "model/processing/reconstruction/common/ScalarGrid3D.h"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
// Classic 256-case table from splashsurf (MIT), encoded as edgeIndex + 1.
constexpr std::string_view kEncodedCases = "AAAAAAAAAAAAAAAAAAAAAAEJBAAAAAAAAAAAAAAAAAABAgoAAAAAAAAAAAAAAAAAAgkECgkCAAAAAAAAAAAAAAIDCwAAAAAAAAAAAAAAAAABCQQCAwsAAAAAAAAAAAAACgMLAQMKAAAAAAAAAAAAAAMJBAMLCQsKCQAAAAAAAAAEDAMAAAAAAAAAAAAAAAAAAQwDCQwBAAAAAAAAAAAAAAIKAQMEDAAAAAAAAAAAAAACDAMCCgwKCQwAAAAAAAAABAsCDAsEAAAAAAAAAAAAAAELAgEJCwkMCwAAAAAAAAAECgEEDAoMCwoAAAAAAAAACgkLCwkMAAAAAAAAAAAAAAUICQAAAAAAAAAAAAAAAAAFBAEIBAUAAAAAAAAAAAAAAQIKCQUIAAAAAAAAAAAAAAUCCgUIAggEAgAAAAAAAAACAwsJBQgAAAAAAAAAAAAABAUIBAEFAgMLAAAAAAAAAAoDCwoBAwkFCAAAAAAAAAADCwoDCggDCAQICgUAAAAACQUIBAwDAAAAAAAAAAAAAAwFCAwDBQMBBQAAAAAAAAAKAQIJBQgDBAwAAAAAAAAABQgMCgUMCgwDCgMCAAAAAAQLAgQMCwgJBQAAAAAAAAACDAsCBQwCAQUIDAUAAAAABQgJCgEMCgwLDAEEAAAAAAUIDAUMCgoMCwAAAAAAAAAKBgUAAAAAAAAAAAAAAAAACgYFAQkEAAAAAAAAAAAAAAEGBQIGAQAAAAAAAAAAAAAJBgUJBAYEAgYAAAAAAAAAAgMLCgYFAAAAAAAAAAAAAAQBCQIDCwUKBgAAAAAAAAAGAwsGBQMFAQMAAAAAAAAAAwsGBAMGBAYFBAUJAAAAAAoGBQMEDAAAAAAAAAAAAAABDAMBCQwFCgYAAAAAAAAAAQYFAQIGAwQMAAAAAAAAAAMCBgMGCQMJDAUJBgAAAAALBAwLAgQKBgUAAAAAAAAABQoGAQkCCQsCCQwLAAAAAAYFAQYBDAYMCwwBBAAAAAAGBQkGCQsLCQwAAAAAAAAACggJBggKAAAAAAAAAAAAAAoEAQoGBAYIBAAAAAAAAAABCAkBAggCBggAAAAAAAAAAgYEBAYIAAAAAAAAAAAAAAoICQoGCAsCAwAAAAAAAAALAgMKBgEGBAEGCAQAAAAACQEDCQMGCQYICwYDAAAAAAMLBgMGBAQGCAAAAAAAAAAICgYICQoEDAMAAAAAAAAACgYICggDCgMBAwgMAAAAAAMEDAECCQIICQIGCAAAAAAMAwIMAggIAgYAAAAAAAAACgYJCQYICwIECwQMAAAAAAYIAQYBCggMAQIBCwwLAQAMCwEMAQQLBgEJAQgGCAEADAsGCAwGAAAAAAAAAAAAAAsHBgAAAAAAAAAAAAAAAAABCQQGCwcAAAAAAAAAAAAACgECBgsHAAAAAAAAAAAAAAIJBAIKCQYLBwAAAAAAAAACBwYDBwIAAAAAAAAAAAAAAgcGAgMHBAEJAAAAAAAAAAoHBgoBBwEDBwAAAAAAAAAGCgkGCQMGAwcEAwkAAAAAAwQMCwcGAAAAAAAAAAAAAAwBCQwDAQsHBgAAAAAAAAABAgoDBAwGCwcAAAAAAAAABgsHAgoDCgwDCgkMAAAAAAcEDAcGBAYCBAAAAAAAAAABCQwBDAYBBgIGDAcAAAAABAwHAQQHAQcGAQYKAAAAAAcGCgcKDAwKCQAAAAAAAAAGCwcFCAkAAAAAAAAAAAAABQQBBQgEBwYLAAAAAAAAAAIKAQYLBwkFCAAAAAAAAAALBwYCCggCCAQICgUAAAAABwIDBwYCBQgJAAAAAAAAAAIDBgYDBwQBBQQFCAAAAAAJBQgKAQYBBwYBAwcAAAAACAQKCAoFBAMKBgoHAwcKAAQMAwgJBQsHBgAAAAAAAAAGCwcFCAMFAwEDCAwAAAAAAQIKBQgJAwQMBgsHAAAAAAoDAgoMAwoFDAgMBQYLBwAJBQgEDAYEBgIGDAcAAAAABgIMBgwHAgEMCAwFAQUMAAEGCgEHBgEEBwwHBAkFCAAHBgoHCgwFCAoIDAoAAAAACwUKBwULAAAAAAAAAAAAAAULBwUKCwEJBAAAAAAAAAALAQILBwEHBQEAAAAAAAAACQQCCQIHCQcFBwILAAAAAAIFCgIDBQMHBQAAAAAAAAAEAQkCAwoDBQoDBwUAAAAAAQMFBQMHAAAAAAAAAAAAAAkEAwkDBQUDBwAAAAAAAAALBQoLBwUMAwQAAAAAAAAAAQkDAwkMBQoLBQsHAAAAAAQMAwECBwEHBQcCCwAAAAAHBQIHAgsFCQIDAgwJDAIACgcFCgQHCgIEDAcEAAAAAAkMAgkCAQwHAgoCBQcFAgAEDAcEBwEBBwUAAAAAAAAABwUJDAcJAAAAAAAAAAAAAAgLBwgJCwkKCwAAAAAAAAABCAQBCwgBCgsHCAsAAAAACwcIAgsIAggJAgkBAAAAAAsHCAsIAgIIBAAAAAAAAAACAwcCBwkCCQoJBwgAAAAAAwcKAwoCBwgKAQoECAQKAAgJAQgBBwcBAwAAAAAAAAAIBAMHCAMAAAAAAAAAAAAAAwQMCwcJCwkKCQcIAAAAAAMBCAMIDAEKCAcICwoLCAACCQECCAkCCwgHCAsDBAwADAMCDAIICwcCBwgCAAAAAAkKBwkHCAoCBwwHBAIEBwABCgIMBwgAAAAAAAAAAAAACAkBCAEHBAwBDAcBAAAAAAgMBwAAAAAAAAAAAAAAAAAIBwwAAAAAAAAAAAAAAAAABAEJDAgHAAAAAAAAAAAAAAECCgwIBwAAAAAAAAAAAAAJAgoJBAIMCAcAAAAAAAAACwIDBwwIAAAAAAAAAAAAAAIDCwQBCQcMCAAAAAAAAAADCgEDCwoHDAgAAAAAAAAABwwIAwsECwkECwoJAAAAAAgDBAcDCAAAAAAAAAAAAAAIAQkIBwEHAwEAAAAAAAAAAwgHAwQIAQIKAAAAAAAAAAIHAwIJBwIKCQkIBwAAAAALCAcLAggCBAgAAAAAAAAACwgHAggLAgkIAgEJAAAAAAEECAEICwELCgcLCAAAAAAIBwsICwkJCwoAAAAAAAAABwkFDAkHAAAAAAAAAAAAAAQHDAQBBwEFBwAAAAAAAAAJBwwJBQcKAQIAAAAAAAAACgUHCgcECgQCDAQHAAAAAAcJBQcMCQMLAgAAAAAAAAACAwsEAQwBBwwBBQcAAAAABQwJBQcMAQMKAwsKAAAAAAsKBAsEAwoFBAwEBwUHBAAJAwQJBQMFBwMAAAAAAAAAAQUDBQcDAAAAAAAAAAAAAAIKAQMEBQMFBwUECQAAAAACCgUCBQMDBQcAAAAAAAAACQIECQcCCQUHBwsCAAAAAAsCAQsBBwcBBQAAAAAAAAAFBwQFBAkHCwQBBAoLCgQACwoFBwsFAAAAAAAAAAAAAAUKBggHDAAAAAAAAAAAAAABCQQFCgYMCAcAAAAAAAAABgECBgUBCAcMAAAAAAAAAAwIBwkEBQQGBQQCBgAAAAAKBgULAgMIBwwAAAAAAAAABwwIAgMLAQkEBQoGAAAAAAgHDAYFCwUDCwUBAwAAAAAEBQkEBgUEAwYLBgMMCAcACAMECAcDBgUKAAAAAAAAAAoGBQEJBwEHAwcJCAAAAAAEBwMECAcCBgEGBQEAAAAABwMJBwkIAwIJBQkGAgYJAAoGBQsCBwIIBwIECAAAAAACBwsCCAcCAQgJCAEKBgUABQELBQsGAQQLBwsIBAgLAAgHCwgLCQYFCwUJCwAAAAAHCgYHDAoMCQoAAAAAAAAABAcMAQcEAQYHAQoGAAAAAAEMCQEGDAECBgYHDAAAAAAHDAQHBAYGBAIAAAAAAAAAAgMLCgYMCgwJDAYHAAAAAAEMBAEHDAEKBwYHCgIDCwAMCQYMBgcJAQYLBgMBAwYABwwEBwQGAwsECwYEAAAAAAYJCgYDCQYHAwQJAwAAAAAKBgcKBwEBBwMAAAAAAAAAAgYJAgkBBgcJBAkDBwMJAAIGBwMCBwAAAAAAAAAAAAACBAcCBwsECQcGBwoJCgcACwIBCwEHCgYBBgcBAAAAAAEECQYHCwAAAAAAAAAAAAALBgcAAAAAAAAAAAAAAAAADAYLCAYMAAAAAAAAAAAAAAwGCwwIBgkEAQAAAAAAAAAGDAgGCwwCCgEAAAAAAAAACwgGCwwICgkCCQQCAAAAAAwCAwwIAggGAgAAAAAAAAABCQQCAwgCCAYIAwwAAAAACggGCgMICgEDAwwIAAAAAAgGAwgDDAYKAwQDCQoJAwADBgsDBAYECAYAAAAAAAAACQMBCQYDCQgGCwMGAAAAAAoBAgYLBAYECAQLAwAAAAAKCQMKAwIJCAMLAwYIBgMAAgQGBAgGAAAAAAAAAAAAAAEJCAEIAgIIBgAAAAAAAAAKAQQKBAYGBAgAAAAAAAAACgkIBgoIAAAAAAAAAAAAAAYJBQYLCQsMCQAAAAAAAAAGAQUGDAEGCwwMBAEAAAAAAQIKCQULCQsMCwUGAAAAAAsMBQsFBgwEBQoFAgQCBQADBgIDCQYDDAkFBgkAAAAAAQUMAQwEBQYMAwwCBgIMAAEDBgEGCgMMBgUGCQwJBgAKBQYDDAQAAAAAAAAAAAAAAwYLBAYDBAUGBAkFAAAAAAYLAwYDBQUDAQAAAAAAAAAECwMEBgsECQYFBgkBAgoABgsDBgMFAgoDCgUDAAAAAAkFBgkGBAQGAgAAAAAAAAABBQYCAQYAAAAAAAAAAAAACQUGCQYECgEGAQQGAAAAAAoFBgAAAAAAAAAAAAAAAAAFDAgFCgwKCwwAAAAAAAAAAQkEBQoICgwICgsMAAAAAAILDAIMBQIFAQgFDAAAAAAEAgUEBQkCCwUIBQwLDAUABQwICgwFCgMMCgIDAAAAAAoIBQoMCAoCDAMMAgEJBAAMCAUMBQMDBQEAAAAAAAAADAgFDAUDCQQFBAMFAAAAAAMKCwMICgMECAgFCgAAAAAKCwgKCAULAwgJCAEDAQgABAgLBAsDCAULAgsBBQELAAILAwkIBQAAAAAAAAAAAAAFCgIFAggIAgQAAAAAAAAABQoCBQIIAQkCCQgCAAAAAAUBBAgFBAAAAAAAAAAAAAAFCQgAAAAAAAAAAAAAAAAACgsJCwwJAAAAAAAAAAAAAAQBCgQKDAwKCwAAAAAAAAABAgsBCwkJCwwAAAAAAAAABAILDAQLAAAAAAAAAAAAAAIDDAIMCgoMCQAAAAAAAAAEAQoECgwCAwoDDAoAAAAAAQMMCQEMAAAAAAAAAAAAAAQDDAAAAAAAAAAAAAAAAAADBAkDCQsLCQoAAAAAAAAACgsDAQoDAAAAAAAAAAAAAAMECQMJCwECCQILCQAAAAACCwMAAAAAAAAAAAAAAAAAAgQJCgIJAAAAAAAAAAAAAAEKAgAAAAAAAAAAAAAAAAABBAkAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==";

std::vector<std::int8_t> decodeCases()
{
    std::array<int, 256> inverse{};
    inverse.fill(-1);
    constexpr std::string_view alphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (std::size_t index = 0; index < alphabet.size(); ++index) {
        inverse[static_cast<unsigned char>(alphabet[index])] = static_cast<int>(index);
    }

    std::vector<std::int8_t> result;
    result.reserve(4096U);
    unsigned accumulator = 0U;
    int bitCount = 0;
    for (const unsigned char character : kEncodedCases) {
        if (character == '=') {
            break;
        }
        const int value = inverse[character];
        if (value < 0) {
            continue;
        }
        accumulator = (accumulator << 6U) | static_cast<unsigned>(value);
        bitCount += 6;
        if (bitCount >= 8) {
            bitCount -= 8;
            const unsigned byte = (accumulator >> static_cast<unsigned>(bitCount)) & 0xffU;
            result.push_back(static_cast<std::int8_t>(static_cast<int>(byte) - 1));
        }
    }
    return result;
}

const std::vector<std::int8_t> &caseTable()
{
    static const std::vector<std::int8_t> table = decodeCases();
    return table;
}

const std::array<Eigen::Vector3i, 8> kCornerOffsets{{
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
    {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
}};

constexpr std::array<std::array<int, 2>, 12> kEdgeCorners{{
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7},
}};

struct GridEdge
{
    std::size_t first = 0U;
    std::size_t second = 0U;
    auto operator<=>(const GridEdge &) const = default;
};

GridEdge gridEdge(std::size_t first, std::size_t second)
{
    return {std::min(first, second), std::max(first, second)};
}

Eigen::Vector3d interpolate(
    const Eigen::Vector3d &first,
    const Eigen::Vector3d &second,
    double firstValue,
    double secondValue,
    double isoValue)
{
    const double denominator = secondValue - firstValue;
    const double amount = std::abs(denominator) <= 1.0e-15
        ? 0.5
        : std::clamp((isoValue - firstValue) / denominator, 0.0, 1.0);
    return first + amount * (second - first);
}
}

ReconstructionMethod MarchingCubesReconstructor::method() const
{
    return ReconstructionMethod::MarchingCubes;
}

ReconstructionRequirements MarchingCubesReconstructor::requirements() const
{
    return {NormalRequirement::NotUsed, InputRepresentation::ScalarField};
}

ReconstructionResult MarchingCubesReconstructor::reconstruct(
    const ReconstructionInput &input,
    const ReconstructionMethodParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    const auto *methodParameters = std::get_if<MarchingCubesParameters>(&parameters);
    if (!methodParameters || !input.scalarGrid) {
        ReconstructionResult result;
        result.errorMessage = !methodParameters
            ? "Marching Cubes received parameters for another method."
            : "Marching Cubes requires an existing scalar grid.";
        return result;
    }
    return extract(*input.scalarGrid, *methodParameters, progressCallback, cancellationToken);
}

ReconstructionResult MarchingCubesReconstructor::extract(
    const ScalarGrid3D &grid,
    const MarchingCubesParameters &parameters,
    const ProgressCallback &progressCallback,
    const CancellationToken &cancellationToken) const
{
    ReconstructionResult result;
    ProgressReporter reporter(progressCallback);
    reporter.report(ProcessingStage::Preparing, 0.0, "Preparing Marching Cubes");
    if (!std::isfinite(parameters.isoValue) || !std::isfinite(parameters.geometricTolerance) ||
        parameters.geometricTolerance <= 0.0) {
        result.errorMessage = "Marching Cubes requires a finite iso-value and positive tolerance.";
        return result;
    }
    if (caseTable().size() != 4096U) {
        result.errorMessage = "Marching Cubes topology table is invalid.";
        return result;
    }

    const Eigen::Vector3i dimensions = grid.dimensions();
    TriangleMesh mesh;
    std::vector<Point3d> vertices;
    std::map<GridEdge, std::size_t> edgeVertices;
    const int sliceCount = dimensions.z() - 1;
    reporter.report(ProcessingStage::BuildingSpatialIndex, 0.05, "Preparing grid-edge cache");

    for (int z = 0; z < sliceCount; ++z) {
        if (cancellationToken.isCancellationRequested()) {
            result.cancelled = true;
            result.errorMessage = "Processing cancelled.";
            reporter.cancelled(result.errorMessage);
            return result;
        }
        for (int y = 0; y < dimensions.y() - 1; ++y) {
            for (int x = 0; x < dimensions.x() - 1; ++x) {
                std::array<Eigen::Vector3d, 8> positions{};
                std::array<double, 8> values{};
                std::array<std::size_t, 8> linearIndices{};
                unsigned caseIndex = 0U;
                for (std::size_t corner = 0; corner < kCornerOffsets.size(); ++corner) {
                    const Eigen::Vector3i coordinate =
                        Eigen::Vector3i(x, y, z) + kCornerOffsets[corner];
                    positions[corner] = grid.position(coordinate.x(), coordinate.y(), coordinate.z());
                    values[corner] = grid.value(coordinate.x(), coordinate.y(), coordinate.z());
                    linearIndices[corner] = *grid.checkedIndex(
                        coordinate.x(), coordinate.y(), coordinate.z());
                    if (values[corner] > parameters.isoValue) {
                        caseIndex |= 1U << corner;
                    }
                }

                const std::size_t tableOffset = static_cast<std::size_t>(caseIndex) * 16U;
                for (std::size_t cursor = 0; cursor < 15U; cursor += 3U) {
                    if (caseTable()[tableOffset + cursor] < 0) {
                        break;
                    }
                    std::array<std::size_t, 3> indices{};
                    for (std::size_t triangleCorner = 0; triangleCorner < 3U; ++triangleCorner) {
                        const int edgeIndex = caseTable()[tableOffset + cursor + triangleCorner];
                        const auto &edge = kEdgeCorners[static_cast<std::size_t>(edgeIndex)];
                        const std::size_t firstCorner = static_cast<std::size_t>(edge[0]);
                        const std::size_t secondCorner = static_cast<std::size_t>(edge[1]);
                        const GridEdge key = gridEdge(
                            linearIndices[firstCorner], linearIndices[secondCorner]);
                        auto existing = edgeVertices.find(key);
                        if (existing == edgeVertices.end()) {
                            const Eigen::Vector3d point = interpolate(
                                positions[firstCorner], positions[secondCorner],
                                values[firstCorner], values[secondCorner], parameters.isoValue);
                            const std::size_t vertexIndex = vertices.size();
                            vertices.emplace_back(point.x(), point.y(), point.z());
                            existing = edgeVertices.emplace(key, vertexIndex).first;
                        }
                        indices[2U - triangleCorner] = existing->second;
                    }
                    mesh.addTriangle(indices[0], indices[1], indices[2]);
                }
            }
        }
        reporter.report(
            ProcessingStage::PropagatingSurface,
            mapPhaseProgress(0.05, 0.92, static_cast<double>(z + 1) / sliceCount),
            "Extracting scalar-field slices");
    }

    mesh.setVertices(std::move(vertices));
    if (mesh.empty()) {
        result.succeeded = true;
        result.mesh = std::move(mesh);
        reporter.report(ProcessingStage::Finalizing, 0.99, "No iso-surface intersects the grid");
        return result;
    }

    reporter.report(ProcessingStage::Finalizing, 0.94, "Validating extracted mesh");
    MeshValidationResult validation = validateMesh(std::move(mesh), parameters.geometricTolerance);
    if (!validation.valid) {
        result.errorMessage = std::move(validation.errorMessage);
        return result;
    }
    result.mesh = std::move(validation.mesh);
    result.diagnostics = validation.diagnostics;
    result.succeeded = true;
    reporter.report(ProcessingStage::Finalizing, 0.99, "Marching Cubes mesh ready");
    return result;
}
