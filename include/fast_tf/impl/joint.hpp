#pragma once

#include <type_traits>

#include <eigen3/Eigen/Dense>

#include "fast_tf/impl/link.hpp"

namespace fast_tf {

struct Null {};

template <internal::is_link ChildT>
struct Joint {
    using Parent = Null;
};

namespace internal {

template <class T>
concept is_non_root_link = is_link<T> && !std::is_same_v<typename Joint<T>::Parent, Null>;

template <typename ParentT, typename ChildT>
concept has_joint = std::is_same_v<ParentT, typename Joint<ChildT>::Parent>
                 && requires(Joint<ChildT> joint) { joint.transform; };

template <typename ParentT, typename ChildT>
concept has_static_joint = has_joint<ParentT, ChildT> && (fast_tf::Joint<ChildT>::transform, true);

template <typename T>
concept is_transform = std::is_same_v<std::remove_cvref_t<T>, Eigen::Isometry3d>;
template <typename ParentT, typename ChildT>
concept has_transform_joint = has_joint<ParentT, ChildT> && requires(Joint<ChildT> joint) {
    { joint.transform } -> is_transform;
};

template <typename T>
concept is_rotation = std::is_same_v<std::remove_cvref_t<T>, Eigen::Quaterniond>
                   || std::is_same_v<std::remove_cvref_t<T>, Eigen::AngleAxisd>;
template <typename ParentT, typename ChildT>
concept has_rotation_joint = has_joint<ParentT, ChildT> && requires(Joint<ChildT> joint) {
    { joint.transform } -> is_rotation;
};

template <typename T>
concept is_translation = std::is_same_v<std::remove_cvref_t<T>, Eigen::Translation3d>;
template <typename ParentT, typename ChildT>
concept has_translation_joint = has_joint<ParentT, ChildT> && requires(Joint<ChildT> joint) {
    { joint.transform } -> is_translation;
};

} // namespace internal

template <internal::is_link From, internal::is_link To>
requires(internal::has_static_joint<From, To>) inline auto& get_transform() {
    return Joint<To>::transform;
}

template <internal::is_link From, internal::is_link To, typename JointCollectionT>
requires(internal::has_joint<From, To> && JointCollectionT::template contains_joint_v<To>)
inline auto& get_transform(JointCollectionT& collection, auto&...) {
    return collection.template get_transform<From, To>();
}

template <
    internal::is_link From, internal::is_link To, typename JointCollectionT,
    typename... JointCollectionTs>
requires(internal::has_joint<From, To> && !JointCollectionT::template contains_joint_v<To>)
inline auto& get_transform(JointCollectionT&, JointCollectionTs&... collections) {
    return get_transform<From, To>(collections...);
}

} // namespace fast_tf