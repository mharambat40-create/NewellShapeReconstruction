#ifndef NEWELL_MODEL_GEOMETRY_TRIANGLEMESH_H
#define NEWELL_MODEL_GEOMETRY_TRIANGLEMESH_H

#include "model/geometry/Point3d.h"

#include <array>
#include <cstddef>
#include <vector>

struct Triangle
{
    std::array<std::size_t, 3> vertexIndices{};
};

class TriangleMesh
{
public:
    using VertexContainer = std::vector<Point3d>;
    using TriangleContainer = std::vector<Triangle>;

    void setVertices(const VertexContainer &vertices);
    void setVertices(VertexContainer &&vertices);
    void addTriangle(std::size_t first, std::size_t second, std::size_t third);
    void clear();

    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::size_t vertexCount() const;
    [[nodiscard]] std::size_t triangleCount() const;
    [[nodiscard]] const VertexContainer &vertices() const;
    [[nodiscard]] const TriangleContainer &triangles() const;
    [[nodiscard]] bool hasValidIndices() const;

private:
    VertexContainer vertices_;
    TriangleContainer triangles_;
};

#endif // NEWELL_MODEL_GEOMETRY_TRIANGLEMESH_H
