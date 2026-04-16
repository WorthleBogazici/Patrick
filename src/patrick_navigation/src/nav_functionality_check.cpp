// nav_functionality_check.cpp — Sequential base movement test (loop)
//
// Tests the Kobuki diff-drive base with a repeating sequence:
//   1. Rotate in place (CW 360°)
//   2. Rotate in place (CCW 360°)
//   3. Drive forward 0.5 m
//   4. Drive backward 0.5 m
//   5. Pause
//
// Publishes Twist on /diff_drive_controller/cmd_vel_unstamped and
// monitors /odom for feedback.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

// Extract yaw from quaternion without tf2 dependency
static double quat_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

class NavFunctionalityCheck : public rclcpp::Node
{
public:
  NavFunctionalityCheck()
  : Node("nav_functionality_check")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_yaw_ = quat_to_yaw(msg->pose.pose.orientation);
        got_odom_ = true;
      });

    // Run the test sequence in a separate thread
    test_thread_ = std::thread(&NavFunctionalityCheck::run_test, this);
  }

  ~NavFunctionalityCheck()
  {
    if (test_thread_.joinable()) {
      test_thread_.join();
    }
  }

private:
  static constexpr double LINEAR_VEL = 0.2;    // m/s
  static constexpr double ANGULAR_VEL = 0.5;   // rad/s
  static constexpr double DRIVE_DIST = 0.5;    // meters
  static constexpr double ROTATE_ANGLE = 2.0 * M_PI;  // full rotation
  static constexpr double PAUSE_SEC = 2.0;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::thread test_thread_;

  std::mutex odom_mutex_;
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  std::atomic<bool> got_odom_{false};

  void get_odom(double & x, double & y, double & yaw)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x = odom_x_;
    y = odom_y_;
    yaw = odom_yaw_;
  }

  void publish_cmd(double linear_x, double angular_z)
  {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_footprint";
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;
    cmd_pub_->publish(msg);
  }

  void stop()
  {
    publish_cmd(0.0, 0.0);
  }

  // Normalize angle to [-pi, pi]
  double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // Rotate by a given angle (positive = CCW, negative = CW)
  void rotate(double angle_rad)
  {
    double start_x, start_y, start_yaw;
    get_odom(start_x, start_y, start_yaw);
    (void)start_x;
    (void)start_y;

    double accumulated = 0.0;
    double prev_yaw = start_yaw;
    double vel = (angle_rad > 0) ? ANGULAR_VEL : -ANGULAR_VEL;

    rclcpp::Rate rate(20);
    while (rclcpp::ok() && std::abs(accumulated) < std::abs(angle_rad)) {
      publish_cmd(0.0, vel);

      double cur_x, cur_y, cur_yaw;
      get_odom(cur_x, cur_y, cur_yaw);

      double delta = normalize_angle(cur_yaw - prev_yaw);
      accumulated += delta;
      prev_yaw = cur_yaw;

      rate.sleep();
    }
    stop();
  }

  // Drive straight by a given distance (positive = forward, negative = backward)
  void drive(double distance_m)
  {
    double start_x, start_y, start_yaw;
    get_odom(start_x, start_y, start_yaw);

    double vel = (distance_m > 0) ? LINEAR_VEL : -LINEAR_VEL;

    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {
      double cur_x, cur_y, cur_yaw;
      get_odom(cur_x, cur_y, cur_yaw);

      double dx = cur_x - start_x;
      double dy = cur_y - start_y;
      double traveled = std::sqrt(dx * dx + dy * dy);

      if (traveled >= std::abs(distance_m)) {
        break;
      }

      publish_cmd(vel, 0.0);
      rate.sleep();
    }
    stop();
  }

  void pause(double seconds)
  {
    stop();
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
  }

  void run_test()
  {
    // Wait for odom
    RCLCPP_INFO(this->get_logger(), "Waiting for odometry...");
    while (rclcpp::ok() && !got_odom_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Odometry received. Starting navigation test loop.");

    int cycle = 0;
    while (rclcpp::ok()) {
      cycle++;
      RCLCPP_INFO(this->get_logger(), "=== Cycle %d ===", cycle);

      // 1. Rotate CW 360°
      RCLCPP_INFO(this->get_logger(), "  Rotating CW 360 degrees...");
      rotate(-ROTATE_ANGLE);
      pause(PAUSE_SEC);

      if (!rclcpp::ok()) break;

      // 2. Rotate CCW 360°
      RCLCPP_INFO(this->get_logger(), "  Rotating CCW 360 degrees...");
      rotate(ROTATE_ANGLE);
      pause(PAUSE_SEC);

      if (!rclcpp::ok()) break;

      // 3. Drive forward 0.5 m
      RCLCPP_INFO(this->get_logger(), "  Driving forward %.1f m...", DRIVE_DIST);
      drive(DRIVE_DIST);
      pause(PAUSE_SEC);

      if (!rclcpp::ok()) break;

      // 4. Drive backward 0.5 m
      RCLCPP_INFO(this->get_logger(), "  Driving backward %.1f m...", DRIVE_DIST);
      drive(-DRIVE_DIST);
      pause(PAUSE_SEC);

      RCLCPP_INFO(this->get_logger(), "=== Cycle %d complete ===", cycle);
    }

    stop();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavFunctionalityCheck>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
