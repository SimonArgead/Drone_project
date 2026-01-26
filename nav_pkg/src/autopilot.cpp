#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

// --- State & helper structs ---

struct StateEstimate {
  double x{0.0}, y{0.0}, z{0.0};
  double vx{0.0}, vy{0.0}, vz{0.0};

  double roll{0.0}, pitch{0.0}, yaw{0.0};
  double roll_rate{0.0}, pitch_rate{0.0}, yaw_rate{0.0};

  double altitude{0.0};
  double airspeed{0.0};

  bool has_odom{false};
  bool has_imu{false};
  bool has_mag{false};
  bool has_baro{false};
};

struct Waypoint {
  double x;
  double y;
  double z;
  double speed;
};

struct NavigationCommand {
  double desired_yaw{0.0};
  double desired_altitude{0.0};
  double desired_speed{0.0};
  bool   done{false};
  bool   loiter{false};
};

struct ActuatorCmd {
  double flap_left{0.0};
  double flap_right{0.0};
  double prop{0.0};
};

// PID controller

class PID {
public:
  PID() = default;
  PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

  void setGains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
  }

  double update(double error, double dt) {
    if (dt <= 0.0) return 0.0;
    integral_ += error * dt;
    double deriv = (error - prev_error_) / dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * deriv;
  }

private:
  double kp_{0.0}, ki_{0.0}, kd_{0.0};
  double integral_{0.0};
  double prev_error_{0.0};
};

// --- Navigator ---

class Navigator {
public:
  void setMission(const std::vector<Waypoint> &wps) {
    waypoints_ = wps;
    idx_ = 0;

    if (!waypoints_.empty()) {
      last_wp_ = waypoints_.back();
      has_last_wp_ = true;
    }
  }

  void setPositionDeadzone(double d) { pos_deadzone_ = d; }
  void setAltitudeDeadzone(double d) { alt_deadzone_ = d; }
  void setYawDeadzone(double d)      { yaw_deadzone_ = d; }

  void requestLanding(bool landing) { landing_requested_ = landing; }

  bool active() const {
    return !waypoints_.empty() && idx_ < waypoints_.size();
  }

  NavigationCommand update(const StateEstimate &st, double dt) {
    NavigationCommand cmd{};

    if (!st.has_odom) {
      return cmd;
    }

    if (active()) {
      const auto &wp = waypoints_[idx_];

      if (reachedWaypoint(st, wp)) {
        idx_++;
        if (!active()) {
          cmd.done = true;
          if (has_last_wp_ && !landing_requested_) {
            applyLoiterLogic(st, cmd, dt);
          }
          return cmd;
        }
      }

      const auto &current = waypoints_[idx_];
      double dx = current.x - st.x;
      double dy = current.y - st.y;

      double bearing = std::atan2(dy, dx);

      cmd.desired_yaw      = bearing;
      cmd.desired_altitude = current.z;
      cmd.desired_speed    = current.speed;
      cmd.done             = false;
      cmd.loiter           = false;
      return cmd;
    }

    cmd.done = true;
    if (has_last_wp_ && !landing_requested_) {
      applyLoiterLogic(st, cmd, dt);
    }

    return cmd;
  }

private:
  static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  bool reachedWaypoint(const StateEstimate &st, const Waypoint &wp) const {
    double dx = wp.x - st.x;
    double dy = wp.y - st.y;
    double dz = wp.z - st.altitude;

    double dist_xy = std::sqrt(dx * dx + dy * dy);
    bool pos_ok = (dist_xy < pos_deadzone_);
    bool alt_ok = (std::fabs(dz) < alt_deadzone_);

    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = wrapAngle(desired_yaw - st.yaw);
    bool yaw_ok = (std::fabs(yaw_error) < yaw_deadzone_);

    return pos_ok && alt_ok && yaw_ok;
  }

  void applyLoiterLogic(const StateEstimate &st, NavigationCommand &cmd, double /*dt*/) const {
    double dx = st.x - last_wp_.x;
    double dy = st.y - last_wp_.y;

    double dist = std::sqrt(dx * dx + dy * dy);
    double radius = loiter_radius_;

    if (dist < 0.1) {
      dx = radius;
      dy = 0.0;
      dist = radius;
    }

    double tx = -dy;
    double ty = dx;

    double bearing = std::atan2(ty, tx);

    cmd.desired_yaw      = bearing;
    cmd.desired_altitude = last_wp_.z;
    cmd.desired_speed    = std::max(last_wp_.speed, loiter_speed_min_);
    cmd.loiter           = true;
  }

  std::vector<Waypoint> waypoints_;
  std::size_t idx_{0};

  Waypoint last_wp_{};
  bool has_last_wp_{false};

  double pos_deadzone_{3.0};
  double alt_deadzone_{3.0};
  double yaw_deadzone_{0.3};

  double loiter_radius_{10.0};
  double loiter_speed_min_{8.0};

  bool landing_requested_{false};
};

// --- Controller ---

class Controller {
public:
  Controller() {
    heading_pid_.setGains(0.8, 0.0, 0.15);
    altitude_pid_.setGains(0.8, 0.0, 0.15);
    speed_pid_.setGains(0.5, 0.0, 0.05);
    roll_pid_.setGains(1.2, 0.0, 0.1);
    pitch_pid_.setGains(1.2, 0.0, 0.1);
  }

  ActuatorCmd update(const StateEstimate &st,
                     const NavigationCommand &nav,
                     double dt) {
    ActuatorCmd cmd{};

    if (dt <= 0.0) return cmd;

    double heading_error = wrapAngle(nav.desired_yaw - st.yaw);
    double desired_roll  = heading_pid_.update(heading_error, dt);

    double alt_error     = nav.desired_altitude - st.altitude;
    double desired_pitch = altitude_pid_.update(alt_error, dt);

    double speed_error   = nav.desired_speed - st.airspeed;
    double throttle_cmd  = speed_pid_.update(speed_error, dt);

    desired_roll  = saturate(desired_roll,  -0.35, 0.35);
    desired_pitch = saturate(desired_pitch, -0.35, 0.35);

    double roll_error  = desired_roll  - st.roll;
    double pitch_error = desired_pitch - st.pitch;

    double roll_cmd  = roll_pid_.update(roll_error, dt);
    double pitch_cmd = pitch_pid_.update(pitch_error, dt);

    double flap_left  = pitch_cmd + roll_cmd;
    double flap_right = pitch_cmd - roll_cmd;

    flap_left  = saturate(flap_left,  -0.524, 0.524);
    flap_right = saturate(flap_right, -0.524, 0.524);

    double prop = saturate(throttle_cmd, 0.0, 1.0);

    cmd.flap_left  = flap_left;
    cmd.flap_right = flap_right;
    cmd.prop       = prop;
    return cmd;
  }

private:
  PID heading_pid_;
  PID altitude_pid_;
  PID speed_pid_;
  PID roll_pid_;
  PID pitch_pid_;

  static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double saturate(double x, double mn, double mx) {
    return std::max(mn, std::min(mx, x));
  }
};

// --- Autopilot mode ---

enum class AutopilotMode {
  STARTUP,
  WAITING_FOR_SENSORS,
  ACTIVE,
  FAILSAFE
};

// --- Main Node ---

class MidLevelAutopilotNode : public rclcpp::Node {
public:
  MidLevelAutopilotNode()
  : Node("mid_level_autopilot") {
    // Parameters
    this->declare_parameter<std::string>("pose_topic", "/zephyr/odom");
    this->declare_parameter<std::string>("imu_topic", "/zephyr/imu/data");
    this->declare_parameter<std::string>("mag_topic", "/zephyr/magnetometer");
    this->declare_parameter<std::string>("baro_topic", "/zephyr/barometer");
    this->declare_parameter<std::string>("mission_topic", "/autopilot/mission");
    this->declare_parameter<std::string>("zephyr_cmd_topic", "/zephyr_controller/command");
    this->declare_parameter<std::string>("gps_topic", "/zephyr/gps/fix");

    this->declare_parameter<double>("flap_gain", 1.0);
    this->declare_parameter<double>("prop_gain", 1.0);
    this->declare_parameter<double>("prop_min_value", 0.0);

    this->declare_parameter<double>("pos_deadzone", 3.0);
    this->declare_parameter<double>("alt_deadzone", 3.0);
    this->declare_parameter<double>("yaw_deadzone", 0.3);

    // Load parameters
    this->get_parameter("pose_topic", pose_topic_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("mag_topic", mag_topic_);
    this->get_parameter("baro_topic", baro_topic_);
    this->get_parameter("mission_topic", mission_topic_);
    this->get_parameter("zephyr_cmd_topic", zephyr_cmd_topic_);
    this->get_parameter("gps_topic", gps_topic_);

    this->get_parameter("flap_gain", flap_gain_);
    this->get_parameter("prop_gain", prop_gain_);
    this->get_parameter("prop_min_value", prop_min_value_);

    double pos_dz, alt_dz, yaw_dz;
    this->get_parameter("pos_deadzone", pos_dz);
    this->get_parameter("alt_deadzone", alt_dz);
    this->get_parameter("yaw_deadzone", yaw_dz);

    navigator_.setPositionDeadzone(pos_dz);
    navigator_.setAltitudeDeadzone(alt_dz);
    navigator_.setYawDeadzone(yaw_dz);

    // Subscriptions
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MidLevelAutopilotNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MidLevelAutopilotNode::imuCallback, this, std::placeholders::_1));

    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MidLevelAutopilotNode::magCallback, this, std::placeholders::_1));

    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      baro_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&MidLevelAutopilotNode::baroCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(10).transient_local().reliable();
    mission_sub_ = create_subscription<nav_msgs::msg::Path>(
        mission_topic_, qos,
        std::bind(&MidLevelAutopilotNode::missionCallback, this, std::placeholders::_1));

    // Publisher
    zephyr_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      zephyr_cmd_topic_, rclcpp::QoS(10));

    last_time_ = now();
    node_start_time_ = last_time_;
    dt_ = 0.02;

    // Initial topic scan (best-effort snapshot)
    checkAvailableTopics();

    // Timer
    timer_ = create_wall_timer(
      20ms, std::bind(&MidLevelAutopilotNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "MidLevelAutopilotNode started.");
  }

private:
  // --- Mission callback ---

  void missionCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    mission_ = *msg;
    has_mission_ = true;
    std::vector<Waypoint> wps;
    wps.reserve(msg->poses.size());

    for (const auto &pose : msg->poses) {
      Waypoint wp;
      wp.x = pose.pose.position.x;
      wp.y = pose.pose.position.y;
      wp.z = pose.pose.position.z;
      wp.speed = 10.0;  // default speed
      wps.push_back(wp);
    }

    navigator_.setMission(wps);

    RCLCPP_INFO(get_logger(),
      "Received mission with %zu waypoints.", wps.size());
  }

  // --- Callbacks ---

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_odom_ = true;
    if (odom_first_msg_time_.nanoseconds() == 0) {
      odom_first_msg_time_ = this->now();
    }
    last_odom_time_ = this->now();
    odom_count_++;

    state_.x = msg->pose.pose.position.x;
    state_.y = msg->pose.pose.position.y;
    state_.z = msg->pose.pose.position.z;

    state_.vx = msg->twist.twist.linear.x;
    state_.vy = msg->twist.twist.linear.y;
    state_.vz = msg->twist.twist.linear.z;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);

    state_.roll  = 0.95 * state_.roll  + 0.05 * r;
    state_.pitch = 0.95 * state_.pitch + 0.05 * p;
    state_.yaw   = 0.95 * state_.yaw   + 0.05 * y;

    state_.airspeed = std::sqrt(state_.vx * state_.vx + state_.vy * state_.vy);

    if (!state_.has_baro) {
      state_.altitude = state_.z;
    }

    state_.has_odom = true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    has_imu_ = true;
    if (imu_first_msg_time_.nanoseconds() == 0) {
      imu_first_msg_time_ = this->now();
    }
    last_imu_time_ = this->now();
    imu_count_++;

    state_.roll_rate  = msg->angular_velocity.x;
    state_.pitch_rate = msg->angular_velocity.y;
    state_.yaw_rate   = msg->angular_velocity.z;

    if (!std::isnan(msg->orientation.x)) {
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      double r, p, y;
      tf2::Matrix3x3(q).getRPY(r, p, y);

      state_.roll  = 0.9 * state_.roll  + 0.1 * r;
      state_.pitch = 0.9 * state_.pitch + 0.1 * p;
      state_.yaw   = 0.98 * (state_.yaw + state_.yaw_rate * dt_) + 0.02 * y;
    } else {
      state_.yaw = state_.yaw + state_.yaw_rate * dt_;
    }

    state_.has_imu = true;
  }

  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    has_mag_ = true;
    if (mag_first_msg_time_.nanoseconds() == 0) {
      mag_first_msg_time_ = this->now();
    }
    last_mag_time_ = this->now();
    mag_count_++;

    double mx = msg->magnetic_field.x;
    double my = msg->magnetic_field.y;

    double mag_yaw = std::atan2(my, mx);

    state_.yaw = 0.98 * state_.yaw + 0.02 * mag_yaw;

    state_.has_mag = true;
  }

  void baroCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    has_baro_ = true;
    if (baro_first_msg_time_.nanoseconds() == 0) {
      baro_first_msg_time_ = this->now();
    }
    last_baro_time_ = this->now();
    baro_count_++;

    double pressure = msg->fluid_pressure;
    double p0 = 101325.0;
    double k  = 0.084;
    double baro_alt = (p0 - pressure) * k;

    state_.altitude = 0.9 * baro_alt + 0.1 * state_.z;

    state_.has_baro = true;
  }

  // --- Diagnostics helper ---

  void diagnoseSensor(
    const std::string &label,
    const std::string &keyword,
    const std::string &expected_topic,
    bool has_data,
    bool &no_data_warned,
    bool &active_reported,
    const rclcpp::Time &first_msg_time,
    const rclcpp::Time &last_msg_time,
    int &msg_count)
  {
    constexpr double STARTUP_GRACE_SEC = 5.0;
    constexpr double OFFLINE_TIMEOUT_SEC = 2.0;

    auto now_t = this->now();
    double since_start = (now_t - node_start_time_).seconds();

    // Under grace period: bare minimum logging
    if (since_start < STARTUP_GRACE_SEC) {
      return;
    }

    auto topics = this->get_topic_names_and_types();
    std::vector<std::string> matches;
    for (auto &t : topics) {
      if (t.first.find(keyword) != std::string::npos) {
        matches.push_back(t.first);
      }
    }

    // If we previously had data and now it's stale -> lost sensor
    if (has_data && last_msg_time.nanoseconds() != 0) {
      double since_last = (now_t - last_msg_time).seconds();
      if (since_last > OFFLINE_TIMEOUT_SEC && !no_data_warned) {
        RCLCPP_WARN(get_logger(),
          "%s data appears to be lost (no messages for %.1f s). Expected topic: %s",
          label.c_str(), since_last, expected_topic.c_str());
        no_data_warned = true;
        active_reported = false;  // allow re-report if it comes back
      }
    }

    // No data yet (or has_data == false)
    if (!has_data && first_msg_time.nanoseconds() == 0) {
      if (!no_data_warned) {
        if (matches.empty()) {
          RCLCPP_WARN(get_logger(),
            "No %s topics found! Autopilot expects: %s",
            label.c_str(), expected_topic.c_str());
        } else {
          RCLCPP_WARN(get_logger(),
            "No %s data received yet! Autopilot expects: %s",
            label.c_str(), expected_topic.c_str());
          RCLCPP_INFO(get_logger(), "Available %s-like topics:", label.c_str());
          for (auto &m : matches) {
            RCLCPP_INFO(get_logger(), "  %s", m.c_str());
          }
        }
        no_data_warned = true;
      }
      return;
    }

    // There IS data – report Hz once when measured
    if (has_data && msg_count > 0 && last_msg_time.nanoseconds() != 0) {
      double dt = (now_t - last_msg_time).seconds();
      if (dt > 0.0) {
        double hz = msg_count / dt;
        if (!active_reported) {
          RCLCPP_INFO(get_logger(),
            "%s is active at %.1f Hz on topic: %s",
            label.c_str(), hz, expected_topic.c_str());
          active_reported = true;
          no_data_warned = false;  // clear previous "no data" warning
        }
        msg_count = 0;
      }
    }
  }

  void checkAvailableTopics()
  {
    RCLCPP_INFO(get_logger(), "Scanning for sensor topics at startup...");

    auto topics = this->get_topic_names_and_types();

    auto printMatches = [&](const std::string &label,
                            const std::string &keyword,
                            const std::string &expected_topic)
    {
      std::vector<std::string> matches;
      for (auto &t : topics) {
        if (t.first.find(keyword) != std::string::npos) {
          matches.push_back(t.first);
        }
      }

      if (matches.empty()) {
        RCLCPP_WARN(get_logger(),
          "No %s topics found at startup. Autopilot expects: %s",
          label.c_str(), expected_topic.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Found %s topics:", label.c_str());
        for (auto &m : matches) {
          RCLCPP_INFO(get_logger(), "  %s", m.c_str());
        }
        RCLCPP_INFO(get_logger(), "Autopilot expects: %s", expected_topic.c_str());
      }
    };

    printMatches("odometry", "odom", pose_topic_);
    printMatches("IMU", "imu", imu_topic_);
    printMatches("magnetometer", "mag", mag_topic_);
    printMatches("barometer", "baro", baro_topic_);
    printMatches("GPS", "gps", gps_topic_);
  }

  // --- Mode update ---

  void updateMode()
  {
    auto now_t = this->now();
    double since_start = (now_t - node_start_time_).seconds();

    bool odom_ok = state_.has_odom;
    bool imu_ok  = state_.has_imu;
    bool baro_ok = state_.has_baro;
    bool mag_ok  = state_.has_mag;
    // GPS kan være optional – vi bruger kun health-diagnose, ikke som hard gate
    bool all_required_ok = odom_ok && imu_ok && baro_ok && mag_ok;

    if (since_start < 5.0) {
      mode_ = AutopilotMode::STARTUP;
      return;
    }

    if (!all_required_ok) {
      mode_ = AutopilotMode::FAILSAFE;
      return;
    }

    if (all_required_ok && has_mission_) {
      mode_ = AutopilotMode::ACTIVE;
    } else {
      mode_ = AutopilotMode::WAITING_FOR_SENSORS;
    }
  }

  // --- Control loop ---

  void controlLoop() {
    auto t = now();
    dt_ = (t - last_time_).seconds();
    last_time_ = t;

    // Sensor-diagnostics (with grace + smart logging)
    diagnoseSensor("odometry", "odom", pose_topic_,
                   has_odom_, odom_no_data_warned_, odom_active_reported_,
                   odom_first_msg_time_, last_odom_time_, odom_count_);

    diagnoseSensor("IMU", "imu", imu_topic_,
                   has_imu_, imu_no_data_warned_, imu_active_reported_,
                   imu_first_msg_time_, last_imu_time_, imu_count_);

    diagnoseSensor("magnetometer", "mag", mag_topic_,
                   has_mag_, mag_no_data_warned_, mag_active_reported_,
                   mag_first_msg_time_, last_mag_time_, mag_count_);

    diagnoseSensor("barometer", "baro", baro_topic_,
                   has_baro_, baro_no_data_warned_, baro_active_reported_,
                   baro_first_msg_time_, last_baro_time_, baro_count_);

    diagnoseSensor("GPS", "gps", gps_topic_,
                   has_gps_, gps_no_data_warned_, gps_active_reported_,
                   gps_first_msg_time_, last_gps_time_, gps_count_);

    // Sensor health summary (throttled)
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Sensor health: odom=%s imu=%s mag=%s baro=%s gps=%s",
      has_odom_ ? "OK" : "MISSING",
      has_imu_  ? "OK" : "MISSING",
      has_mag_  ? "OK" : "MISSING",
      has_baro_ ? "OK" : "MISSING",
      has_gps_  ? "OK" : "MISSING");

    // Update mode
    updateMode();

    // Behaviour per mode
    switch (mode_) {
      case AutopilotMode::STARTUP:
      case AutopilotMode::WAITING_FOR_SENSORS:
      case AutopilotMode::FAILSAFE: {
        // Simple safe fallback: neutral flaps, light throttle
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data = {0.0, 0.0, 0.4};
        zephyr_pub_->publish(cmd);

        if (!has_odom_) {
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Waiting for odom in mode %d...", static_cast<int>(mode_));
        }
        return;
      }

      case AutopilotMode::ACTIVE:
      default:
        break;
    }

    // ACTIVE mode: require odom state
    if (!state_.has_odom) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                            "ACTIVE mode but waiting for odom state...");
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = {0.0, 0.0, 0.4};
      zephyr_pub_->publish(cmd);
      return;
    }

    NavigationCommand nav_cmd = navigator_.update(state_, dt_);
    ActuatorCmd act = controller_.update(state_, nav_cmd, dt_);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(3);

    double flap_left  = flap_gain_ * act.flap_left;
    double flap_right = flap_gain_ * act.flap_right;
    double prop = prop_gain_ * act.prop;
    if (prop_min_value_ > 0.0 && prop < prop_min_value_) {
      prop = prop_min_value_;
    }
    msg.data[0] = flap_left;
    msg.data[1] = flap_right;
    msg.data[2] = prop;

    zephyr_pub_->publish(msg);
  }

  // ----- Members -----

  std::string pose_topic_;
  std::string imu_topic_;
  std::string mag_topic_;
  std::string baro_topic_;
  std::string mission_topic_;
  std::string zephyr_cmd_topic_;
  std::string gps_topic_;

  double flap_gain_{1.0};
  double prop_gain_{1.0};
  double prop_min_value_{0.0};

  StateEstimate state_;
  Navigator navigator_;
  Controller controller_;

  AutopilotMode mode_{AutopilotMode::STARTUP};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mission_sub_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr zephyr_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  rclcpp::Time node_start_time_;
  double dt_{0.02};

  // Mission
  nav_msgs::msg::Path mission_;
  bool has_mission_ = false;

  // Sensor state flags (raw subscription level)
  bool has_odom_ = false;
  bool has_imu_ = false;
  bool has_mag_ = false;
  bool has_baro_ = false;
  bool has_gps_ = false;  // set when you add GPS callback if needed

  // Diagnostics flags: "no data" + "active reported"
  bool odom_no_data_warned_ = false;
  bool imu_no_data_warned_  = false;
  bool mag_no_data_warned_  = false;
  bool baro_no_data_warned_ = false;
  bool gps_no_data_warned_  = false;

  bool odom_active_reported_ = false;
  bool imu_active_reported_  = false;
  bool mag_active_reported_  = false;
  bool baro_active_reported_ = false;
  bool gps_active_reported_  = false;

  // Timestamps
  rclcpp::Time odom_first_msg_time_;
  rclcpp::Time imu_first_msg_time_;
  rclcpp::Time mag_first_msg_time_;
  rclcpp::Time baro_first_msg_time_;
  rclcpp::Time gps_first_msg_time_;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_mag_time_;
  rclcpp::Time last_baro_time_;
  rclcpp::Time last_gps_time_;

  // Counters for Hz
  int odom_count_ = 0;
  int imu_count_ = 0;
  int mag_count_ = 0;
  int baro_count_ = 0;
  int gps_count_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MidLevelAutopilotNode>());
  rclcpp::shutdown();
  return 0;
}
