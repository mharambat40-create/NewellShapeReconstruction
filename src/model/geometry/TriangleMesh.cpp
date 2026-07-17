#include "model/geometry/TriangleMesh.h"

#include <utility>

void TriangleMesh::setVertices(const VertexContainer &vertices)
{
    vertices_ = vertices;
}

void TriangleMesh::setVertices(VertexContainer &&vertices)
{
    vertices_ = std::move(vertices);
}

void TriangleMesh::addTriangle(std::size_t first, std::size_t second, std::size_t third)
{
    triangles_.push_back(Triangle{{first, second, third}});
}

void TriangleMesh::clear()
{
    vertices_.clear();
    triangles_.clear();
}

bool TriangleMesh::empty() const
{
    return triangles_.empty();
}

std::size_t TriangleMesh::vertexCount() const
{
    return vertices_.size();
}

std::size_t TriangleMesh::triangleCount() const
{
    return triangles_.size();
}

const TriangleMesh::VertexContainer &TriangleMesh::vertices() const
{
    return vertices_;
}

const TriangleMesh::TriangleContainer &TriangleMesh::triangles() const
{
    return triangles_;
}

bool TriangleMesh::hasValidIndices() const
{
    for (const Triangle &triangle : triangles_) {
        const auto &indices = triangle.vertexIndices;
        if (indices[0] >= vertices_.size() ||
            indices[1] >= vertices_.size() ||
            indices[2] >= vertices_.size() ||
            indices[0] == indices[1] ||
            indices[1] == indices[2] ||
            indices[2] == indices[0]) {
            return false;
        }
    }
    return true;
}
