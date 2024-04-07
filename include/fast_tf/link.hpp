#pragma once

#include <type_traits>

#include <eigen3/Eigen/Dense>

namespace fast_tf {

template <typename LinkT>
struct Link {
    using LinkType = LinkT;

    struct Position {
        using LinkType = LinkT;

        Position()
            : position(Eigen::Vector3d::Identity()) {}
        explicit Position(const Eigen::Vector3d& position)
            : position(position) {}
        Position(const double& x, const double& y, const double& z)
            : position{x, y, z} {}

        Eigen::Vector3d& operator*() { return position; }
        const Eigen::Vector3d& operator*() const { return position; }

        Eigen::Vector3d* operator->() { return &position; }
        const Eigen::Vector3d* operator->() const { return &position; }

        Eigen::Vector3d position;
    };

    struct DirectionVector {
        using LinkType = LinkT;

        DirectionVector()
            : vector(Eigen::Vector3d::UnitX()) {}
        explicit DirectionVector(const Eigen::Vector3d& vector)
            : vector(vector) {}
        DirectionVector(const double& x, const double& y, const double& z)
            : vector(x, y, z) {}

        Eigen::Vector3d& operator*() { return vector; }
        const Eigen::Vector3d& operator*() const { return vector; }

        Eigen::Vector3d* operator->() { return &vector; }
        const Eigen::Vector3d* operator->() const { return &vector; }

        Eigen::Vector3d vector;
    };

    struct Rotation {
        using LinkType = LinkT;

        Rotation()
            : quaternion(Eigen::Quaterniond::Identity()) {}
        explicit Rotation(const Eigen::Quaterniond& quaternion)
            : quaternion(quaternion) {}
        explicit Rotation(const Eigen::AngleAxisd& angle_axis)
            : quaternion(angle_axis) {}
        Rotation(const Eigen::Matrix3d& matrix)
            : quaternion(matrix) {}
        Rotation(const double& w, const double& x, const double& y, const double& z)
            : quaternion(w, x, y, z) {}

        Eigen::Quaterniond& operator*() { return quaternion; }
        const Eigen::Quaterniond& operator*() const { return quaternion; }

        Eigen::Quaterniond* operator->() { return &quaternion; }
        const Eigen::Quaterniond* operator->() const { return &quaternion; }

        Eigen::Quaterniond quaternion;
    };
};

namespace internal {
template <class T>
concept is_link = std::is_convertible_v<T, Link<typename T::LinkType>>;
} // namespace internal

} // namespace fast_tf