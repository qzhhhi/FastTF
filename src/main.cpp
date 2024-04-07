#include <eigen3/Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>

#include <fast_tf/fast_tf.hpp>
#include <fast_tf/rcl.hpp>

struct Odom : fast_tf::Link<Odom> {
    static constexpr char name[] = "odom";
};

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct YawLink : fast_tf::Link<YawLink> {
    static constexpr char name[] = "yaw_link";
};

template <>
struct fast_tf::Joint<BaseLink> {
    using Parent = Odom;

    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
};

template <>
struct fast_tf::Joint<YawLink> {
    using Parent = BaseLink;

    Eigen::Translation3d transform = Eigen::Translation3d::Identity();
    // void set_state(double angle) { transform.angle() = angle; }
};

using MyJointCollection = fast_tf::JointCollection<BaseLink, YawLink>;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    MyJointCollection collection;

    collection.set_transform<Odom, BaseLink>(Eigen::Translation3d{0.5, 0, 0});
    // collection.set_transform<Odom, BaseLink>(Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitX()});
    collection.set_transform<BaseLink, YawLink>(Eigen::Translation3d{0, 0.3, 0});

    double state = 0.5;
    while (rclcpp::ok()) {
        fast_tf::rcl::broadcast<Odom, BaseLink>(collection);
        fast_tf::rcl::broadcast<BaseLink, YawLink>(collection);

        state += 0.05;
        Odom::Rotation origin{1, 0, 0, 0};
        auto origin2 = fast_tf::cast<YawLink>(origin, collection);

        auto marker_pub =
            fast_tf::rcl::Node::get_instance().create_publisher<visualization_msgs::msg::Marker>(
                "/gimbal/marker", rclcpp::QoS(1));
        auto aiming_point_             = std::make_unique<visualization_msgs::msg::Marker>();
        aiming_point_->header.frame_id = "yaw_link";
        aiming_point_->type            = visualization_msgs::msg::Marker::SPHERE;
        aiming_point_->action          = visualization_msgs::msg::Marker::ADD;
        aiming_point_->scale.x = aiming_point_->scale.y = aiming_point_->scale.z = 0.05;
        aiming_point_->color.r                                                   = 1.0;
        aiming_point_->color.g                                                   = 0.0;
        aiming_point_->color.b                                                   = 0.0;
        aiming_point_->color.a                                                   = 1.0;
        aiming_point_->lifetime        = rclcpp::Duration::from_seconds(0.1);
        aiming_point_->header.stamp    = fast_tf::rcl::Node::get_instance().now();
        aiming_point_->pose.position.x = origin2->x();
        aiming_point_->pose.position.y = origin2->y();
        aiming_point_->pose.position.z = origin2->z();
        marker_pub->publish(std::move(aiming_point_));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}