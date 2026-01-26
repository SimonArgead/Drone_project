#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class ZephyrActuatorController : public rclcpp::Node
{
public:
  ZephyrActuatorController()
  : Node("zephyr_actuator_controller")
  {
    // Parameters (so you can remap easily later if needed)
    this->declare_parameter<std::string>("command_topic", "/zephyr_controller/command");
    this->declare_parameter<std::string>("flap_left_topic", "/zephyr/joint/flap_left/cmd_pos");
    this->declare_parameter<std::string>("flap_right_topic", "/zephyr/joint/flap_right/cmd_pos");
    this->declare_parameter<std::string>("prop_topic", "/zephyr/joint/propeller/cmd_vel");

    this->get_parameter("command_topic", command_topic_);
    this->get_parameter("flap_left_topic", flap_left_topic_);
    this->get_parameter("flap_right_topic", flap_right_topic_);
    this->get_parameter("prop_topic", prop_topic_);

    // Publishers
    flap_left_pub_  = this->create_publisher<std_msgs::msg::Float64>(flap_left_topic_, 10);
    flap_right_pub_ = this->create_publisher<std_msgs::msg::Float64>(flap_right_topic_, 10);
    prop_pub_       = this->create_publisher<std_msgs::msg::Float64>(prop_topic_, 10);

    // Subscriber
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, 10,
      std::bind(&ZephyrActuatorController::commandCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "ZephyrActuatorController started. Listening on [%s]",
      command_topic_.c_str());
  }

private:
  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received command with size %zu, expected at least 3 (flap_left, flap_right, prop)",
        msg->data.size());
      return;
    }

    // Extract commands
    double flap_left_cmd  = msg->data[0];
    double flap_right_cmd = msg->data[1];
    double throttle       = msg->data[2];   // 0.0–1.0 from autopilot

    // Clamp flaps
    flap_left_cmd  = clamp(flap_left_cmd,  -0.524, 0.524);
    flap_right_cmd = clamp(flap_right_cmd, -0.524, 0.524);

    // Convert throttle → propeller angular velocity
    double max_vel = 300.0;                 // rad/s (tune as needed)
    double prop_vel = throttle * max_vel;
    prop_vel = clamp(prop_vel, 0.0, max_vel);

    // Prepare messages
    std_msgs::msg::Float64 flap_left_msg;
    std_msgs::msg::Float64 flap_right_msg;
    std_msgs::msg::Float64 prop_msg;

    flap_left_msg.data  = flap_left_cmd;
    flap_right_msg.data = flap_right_cmd;
    prop_msg.data       = prop_vel;

    // Publish
    flap_left_pub_->publish(flap_left_msg);
    flap_right_pub_->publish(flap_right_msg);
    prop_pub_->publish(prop_msg);
  }

  static double clamp(double x, double mn, double mx)
  {
    return std::max(mn, std::min(mx, x));
  }

  std::string command_topic_;
  std::string flap_left_topic_;
  std::string flap_right_topic_;
  std::string prop_topic_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr flap_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr flap_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr prop_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZephyrActuatorController>());
  rclcpp::shutdown();
  return 0;
}
