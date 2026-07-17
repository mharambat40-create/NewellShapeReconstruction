#ifndef NEWELL_MODEL_GEOMETRY_NORMALFIELD_H
#define NEWELL_MODEL_GEOMETRY_NORMALFIELD_H

#include <Eigen/Core>

#include <cstddef>
#include <vector>

class Normal3d
{
public:
    Normal3d() = default;
    explicit Normal3d(
        const Eigen::Vector3d &direction,
        bool valid = true,
        double confidence = 1.0);

    [[nodiscard]] double x() const;
    [[nodiscard]] double y() const;
    [[nodiscard]] double z() const;
    [[nodiscard]] const Eigen::Vector3d &vector() const;
    [[nodiscard]] bool isValid() const;
    [[nodiscard]] double confidence() const;

private:
    Eigen::Vector3d direction_ = Eigen::Vector3d::Zero();
    bool valid_ = false;
    double confidence_ = 0.0;
};

class NormalField
{
public:
    void reserve(std::size_t normalCapacity);
    void addNormal(const Normal3d &normal);
    void addNormal(const Eigen::Vector3d &direction, double confidence = 1.0);
    void addInvalidNormal();
    void clear();
    void setConsistentlyOriented(bool consistentlyOriented);

    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::size_t validNormalCount() const;
    [[nodiscard]] const Normal3d &normal(std::size_t index) const;
    [[nodiscard]] const std::vector<Normal3d> &normals() const;
    [[nodiscard]] bool consistentlyOriented() const;

private:
    std::vector<Normal3d> normals_;
    std::size_t validNormalCount_ = 0U;
    bool consistentlyOriented_ = false;
};

#endif // NEWELL_MODEL_GEOMETRY_NORMALFIELD_H
