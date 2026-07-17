#ifndef NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_SCALARGRID3D_H
#define NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_SCALARGRID3D_H

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

class ScalarGrid3D
{
public:
    struct CreateResult;

    [[nodiscard]] static CreateResult create(
        const Eigen::Vector3i &dimensions,
        const Eigen::Vector3d &origin,
        double spacing,
        std::size_t maximumVoxelCount,
        float initialValue = 0.0F);

    [[nodiscard]] const Eigen::Vector3i &dimensions() const;
    [[nodiscard]] const Eigen::Vector3d &origin() const;
    [[nodiscard]] double spacing() const;
    [[nodiscard]] std::size_t voxelCount() const;
    [[nodiscard]] bool contains(int x, int y, int z) const;
    [[nodiscard]] std::optional<std::size_t> checkedIndex(int x, int y, int z) const;
    [[nodiscard]] float value(int x, int y, int z) const;
    [[nodiscard]] float &value(int x, int y, int z);
    [[nodiscard]] Eigen::Vector3d position(int x, int y, int z) const;
    [[nodiscard]] const std::vector<float> &values() const;
    [[nodiscard]] std::vector<float> &values();

private:
    ScalarGrid3D(
        Eigen::Vector3i dimensions,
        Eigen::Vector3d origin,
        double spacing,
        std::vector<float> values);

    Eigen::Vector3i dimensions_ = Eigen::Vector3i::Zero();
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
    double spacing_ = 0.0;
    std::vector<float> values_;
};

struct ScalarGrid3D::CreateResult
{
    std::optional<ScalarGrid3D> grid;
    std::string errorMessage;
};

#endif // NEWELL_MODEL_PROCESSING_RECONSTRUCTION_COMMON_SCALARGRID3D_H
