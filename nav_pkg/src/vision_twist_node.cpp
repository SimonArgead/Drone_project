#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class VisionTwistNode : public rclcpp::Node
{
public:
    VisionTwistNode()
    : Node("vision_twist_node")
    {
        using std::placeholders::_1;

        // Subscription (Kilted: SensorDataQoS anbefales)
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&VisionTwistNode::odomCallback, this, _1));

        // Publisher (almindelig QoS er fint)
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rtabmap/vision_twist",
            rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "vision_twist_node started");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header = msg->header;   // behold timestamp + frame_id
        twist_msg.twist = msg->twist.twist;

        pub_->publish(twist_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionTwistNode>());
    rclcpp::shutdown();
    return 0;
}
