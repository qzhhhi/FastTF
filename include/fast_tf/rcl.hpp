#pragma once

#include <rclcpp/node.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "fast_tf/impl/joint.hpp"

namespace fast_tf {

namespace internal {

template <internal::is_transform TransformT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const TransformT& transform) {
    return {
        static_cast<Eigen::Translation3d>(transform.translation()),
        static_cast<Eigen::Quaterniond>(transform.linear())};
}

template <internal::is_translation TranslationT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const TranslationT& translation) {
    return {static_cast<Eigen::Translation3d>(translation), Eigen::Quaterniond::Identity()};
}

template <internal::is_rotation RotationT>
inline std::tuple<Eigen::Translation3d, Eigen::Quaterniond>
    extract_translation_rotation(const RotationT& rotation) {
    return {Eigen::Translation3d::Identity(), static_cast<Eigen::Quaterniond>(rotation)};
}

} // namespace internal

namespace rcl {

class Node : public rclcpp::Node {
public:
    Node(Node const&)           = delete;
    void operator=(Node const&) = delete;

    static Node& get_instance() {
        static Node instance;
        return instance;
    }

    void tf_broadcast(
        const char* header, const char* child, const Eigen::Translation3d& translation,
        const Eigen::Quaterniond& rotation) {

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = get_clock()->now();
        t.header.frame_id = header;
        t.child_frame_id  = child;

        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();

        t.transform.rotation.w = rotation.w();
        t.transform.rotation.x = rotation.x();
        t.transform.rotation.y = rotation.y();
        t.transform.rotation.z = rotation.z();

        tf_broadcaster_.sendTransform(t);
    }

private:
    Node()
        : rclcpp::Node("fast_tf", rclcpp::NodeOptions().use_intra_process_comms(true))
        , tf_broadcaster_(this) {}

    template <typename From, typename To>
    requires(internal::is_link<From> && internal::is_link<To>) class BroadcastCache {
    public:
    private:
    };

    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

template <internal::is_link From, internal::is_link To, typename... JointCollectionTs>
requires(internal::has_joint<From, To> && requires(const JointCollectionTs&... collections) {
    get_transform<From, To>(collections...);
}) inline void broadcast(const JointCollectionTs&... collections) {
    auto transform               = get_transform<From, To>(collections...);
    auto [translation, rotation] = internal::extract_translation_rotation(transform);
    Node::get_instance().tf_broadcast(From::name, To::name, translation, rotation);
}

template <typename JointCollectionT>
inline void broadcast_all(const JointCollectionT& collection) {
    collection.for_each(
        [&collection]<typename From, typename To>() { broadcast<From, To>(collection); });
}

template <typename JointCollectionT>
inline void broadcast_all_modified(const JointCollectionT& collection) {
    collection.for_each_modified(
        [&collection]<typename From, typename To>() { broadcast<From, To>(collection); });
}

} // namespace rcl

} // namespace fast_tf