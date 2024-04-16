#include <eigen3/Eigen/Dense>
#include <visualization_msgs/msg/marker.hpp>

#include <fast_tf/fast_tf.hpp>
#include <fast_tf/rcl.hpp>

struct BaseLink : fast_tf::Link<BaseLink> {
    static constexpr char name[] = "base_link";
};

struct YawLink : fast_tf::Link<YawLink> {
    static constexpr char name[] = "yaw_link";
};
template <>
struct fast_tf::Joint<YawLink> {
    using Parent                = BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d{
        Eigen::Translation3d{0, 0, 0.32059}
    };
    void set_state(double angle) {
        auto rotation      = Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitZ()};
        transform.linear() = rotation.matrix();
    }
};

struct PitchLink : fast_tf::Link<PitchLink> {
    static constexpr char name[] = "pitch_link";
};
template <>
struct fast_tf::Joint<PitchLink> {
    using Parent                = YawLink;
    Eigen::AngleAxisd transform = {0, Eigen::Vector3d::UnitY()};
    void set_state(double angle) { transform.angle() = angle; }
};

struct LeftFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_front_wheel_link";
};
template <>
struct fast_tf::Joint<LeftFrontWheelLink> {
    using Parent                = BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d{
        Eigen::Translation3d{0.15897, 0.15897, 0}
    };
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

struct LeftBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "left_back_wheel_link";
};
template <>
struct fast_tf::Joint<LeftBackWheelLink> {
    using Parent                = BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d{
        Eigen::Translation3d{-0.15897, 0.15897, 0}
    };
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

struct RightBackWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_back_wheel_link";
};
template <>
struct fast_tf::Joint<RightBackWheelLink> {
    using Parent                = BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d{
        Eigen::Translation3d{-0.15897, -0.15897, 0}
    };
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4 * 3, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

struct RightFrontWheelLink : fast_tf::Link<LeftFrontWheelLink> {
    static constexpr char name[] = "right_front_wheel_link";
};
template <>
struct fast_tf::Joint<RightFrontWheelLink> {
    using Parent                = BaseLink;
    Eigen::Isometry3d transform = Eigen::Isometry3d{
        Eigen::Translation3d{0.15897, -0.15897, 0}
    };
    void set_state(double angle) {
        auto rotation = Eigen::AngleAxisd{-std::numbers::pi / 4, Eigen::Vector3d::UnitZ()}
                      * Eigen::AngleAxisd{angle, Eigen::Vector3d::UnitX()};
        transform.linear() = rotation.matrix();
    }
};

using MyJointCollection = fast_tf::JointCollection<
    YawLink, PitchLink, LeftFrontWheelLink, LeftBackWheelLink, RightBackWheelLink,
    RightFrontWheelLink>;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    MyJointCollection collection;

    double state = 0;
    while (rclcpp::ok()) {
        state += 0.02;
        collection.set_state<BaseLink, YawLink>(state + 0.6);
        collection.set_state<YawLink, PitchLink>(std::abs(std::fmod(state / 2, 0.6) - 0.3) - 0.1);
        collection.set_state<BaseLink, LeftFrontWheelLink>(2 * state + 0.1);
        collection.set_state<BaseLink, LeftBackWheelLink>(2 * state + 0.2);
        collection.set_state<BaseLink, RightBackWheelLink>(2 * state + 0.3);
        collection.set_state<BaseLink, RightFrontWheelLink>(2 * state + 0.4);

        fast_tf::rcl::broadcast<BaseLink, YawLink>(collection);
        fast_tf::rcl::broadcast<YawLink, PitchLink>(collection);
        fast_tf::rcl::broadcast<BaseLink, LeftFrontWheelLink>(collection);
        fast_tf::rcl::broadcast<BaseLink, LeftBackWheelLink>(collection);
        fast_tf::rcl::broadcast<BaseLink, RightBackWheelLink>(collection);
        fast_tf::rcl::broadcast<BaseLink, RightFrontWheelLink>(collection);

        // auto marker_pub =
        //     fast_tf::rcl::Node::get_instance().create_publisher<visualization_msgs::msg::Marker>(
        //         "/gimbal/marker", rclcpp::QoS(1));
        // auto aiming_point_             = std::make_unique<visualization_msgs::msg::Marker>();
        // aiming_point_->header.frame_id = "yaw_link";
        // aiming_point_->type            = visualization_msgs::msg::Marker::SPHERE;
        // aiming_point_->action          = visualization_msgs::msg::Marker::ADD;
        // aiming_point_->scale.x = aiming_point_->scale.y = aiming_point_->scale.z = 0.05;
        // aiming_point_->color.r                                                   = 1.0;
        // aiming_point_->color.g                                                   = 0.0;
        // aiming_point_->color.b                                                   = 0.0;
        // aiming_point_->color.a                                                   = 1.0;
        // aiming_point_->lifetime        = rclcpp::Duration::from_seconds(0.1);
        // aiming_point_->header.stamp    = fast_tf::rcl::Node::get_instance().now();
        // aiming_point_->pose.position.x = origin2->x();
        // aiming_point_->pose.position.y = origin2->y();
        // aiming_point_->pose.position.z = origin2->z();
        // marker_pub->publish(std::move(aiming_point_));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}