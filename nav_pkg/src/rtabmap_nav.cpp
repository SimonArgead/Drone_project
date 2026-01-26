#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RTABMapNavNode : public rclcpp::Node {
public:
  RTABMapNavNode()
  : Node("rtabmap_nav")
  {
    // --- Declare parameters (Jazzy style) ---
    this->declare_parameter<std::string>("pose_topic", "/odom");
    this->declare_parameter<std::string>("mission_topic", "/mission");

    this->declare_parameter<double>("scan_width", 20.0);
    this->declare_parameter<double>("scan_height", 20.0);
    this->declare_parameter<double>("scan_step", 2.0);
    this->declare_parameter<double>("scan_altitude", 5.0);
    this->declare_parameter<double>("scan_speed", 1.0);
    this->declare_parameter<double>("takeoff_speed", 1.0);

    // --- Read parameters ---
    pose_topic_     = this->get_parameter("pose_topic").as_string();
    mission_topic_  = this->get_parameter("mission_topic").as_string();

    scan_width_     = this->get_parameter("scan_width").as_double();
    scan_height_    = this->get_parameter("scan_height").as_double();
    scan_step_      = this->get_parameter("scan_step").as_double();
    scan_altitude_  = this->get_parameter("scan_altitude").as_double();
    scan_speed_     = this->get_parameter("scan_speed").as_double();
    takeoff_speed_  = this->get_parameter("takeoff_speed").as_double();

    // --- Odom subscriber ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RTABMapNavNode::odomCallback, this, std::placeholders::_1));

    // --- Mission publisher ---
    auto qos = rclcpp::QoS(10).transient_local().reliable();
    mission_pub_ = this->create_publisher<nav_msgs::msg::Path>(mission_topic_, qos);

    RCLCPP_INFO(this->get_logger(),
      "RTABMapNavNode started. Generating mission: %.0fm x %.0fm @ %.0fm.",
      scan_width_, scan_height_, scan_altitude_);

    publishMission();
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;
    last_z_ = msg->pose.pose.position.z;
  }

  geometry_msgs::msg::PoseStamped makePose(double x, double y, double z) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void publishMission() {
    nav_msgs::msg::Path mission;
    mission.header.stamp = this->get_clock()->now();
    mission.header.frame_id = "map";

    // Takeoff
    mission.poses.push_back(makePose(0.0, 0.0, scan_altitude_));

    // Lawnmower pattern
    bool left_to_right = true;

    for (double y = 0.0; y <= scan_height_; y += scan_step_) {
      double x1 = left_to_right ? 0.0 : scan_width_;
      double x2 = left_to_right ? scan_width_ : 0.0;

      mission.poses.push_back(makePose(x1, y, scan_altitude_));
      mission.poses.push_back(makePose(x2, y, scan_altitude_));

      left_to_right = !left_to_right;
    }

    mission_pub_->publish(mission);

    RCLCPP_INFO(this->get_logger(),
      "Published mission with %zu poses (1 climb + %zu scan).",
      mission.poses.size(),
      mission.poses.size() > 0 ? mission.poses.size() - 1 : 0);
  }

  // Parameters
  std::string pose_topic_;
  std::string mission_topic_;

  double scan_width_;
  double scan_height_;
  double scan_step_;
  double scan_altitude_;
  double scan_speed_;
  double takeoff_speed_;

  // State
  double last_x_{0.0}, last_y_{0.0}, last_z_{0.0};

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTABMapNavNode>());
  rclcpp::shutdown();
  return 0;
}
