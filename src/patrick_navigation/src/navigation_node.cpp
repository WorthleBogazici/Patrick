// =============================================================================
// navigation_node.cpp — Lifecycle node exposing Kobuki base-motion services
// =============================================================================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "patrick_navigation/kobuki_operations.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NavigationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NavigationNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("navigation_node", opts)
  {
    // Declare parameters (defaults from kobuki_ops)
    declare_parameter("cmd_vel_topic", "/diff_drive_controller/cmd_vel_unstamped");
    declare_parameter("linear_kp", 1.8);
    declare_parameter("linear_ki", 0.08);
    declare_parameter("linear_kd", 0.12);
    declare_parameter("angular_kp", 3.0);
    declare_parameter("angular_ki", 0.05);
    declare_parameter("angular_kd", 0.20);
    declare_parameter("max_linear_velocity", 0.25);
    declare_parameter("min_linear_velocity", 0.02);
    declare_parameter("max_angular_velocity", 1.2);
    declare_parameter("distance_tolerance", 0.02);
    declare_parameter("yaw_tolerance", 0.02);
    declare_parameter("stop_hold_time", 0.20);
    declare_parameter("yaw_stop_hold_time", 0.20);
    declare_parameter("linear_integral_max", 0.40);
    declare_parameter("angular_integral_max", 0.50);
    declare_parameter("max_total_time", 20.0);
    declare_parameter("control_rate", 50.0);
  }

  // ── Lifecycle callbacks ─────────────────────────────────────────────────
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring NavigationNode");

    load_params();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cfg_.cmd_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_.update(msg); });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "NavigationNode active");
    cmd_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "NavigationNode deactivated");
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    cmd_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    cmd_pub_.reset();
    odom_sub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  // ── Public API ────────────────────────────────────────────────────────────
  bool move_forward(double distance_m)
  {
    auto raw = get_raw_publisher();
    if (!raw) return false;
    return kobuki_ops::move_forward(
      shared_from_this()->get_node_base_interface()->get_shared_rcl_node_handle()
        ? std::dynamic_pointer_cast<rclcpp::Node>(std::shared_ptr<rclcpp::Node>())
        : nullptr,
      raw, odom_, distance_m, cfg_);
  }

  kobuki_ops::OdomCache & odom() { return odom_; }
  const kobuki_ops::MovePIDConfig & move_pid_cfg() const { return cfg_; }
  const kobuki_ops::MoveToConfig & move_to_cfg() const { return mt_cfg_; }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr get_raw_publisher()
  {
    return std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(cmd_pub_);
  }

private:
  void load_params()
  {
    double rate = get_parameter("control_rate").as_double();
    cfg_.dt_s = 1.0 / std::max(1.0, rate);
    cfg_.Kp = get_parameter("linear_kp").as_double();
    cfg_.Ki = get_parameter("linear_ki").as_double();
    cfg_.Kd = get_parameter("linear_kd").as_double();
    cfg_.KyawP = get_parameter("angular_kp").as_double();
    cfg_.KyawI = get_parameter("angular_ki").as_double();
    cfg_.KyawD = get_parameter("angular_kd").as_double();
    cfg_.v_max = get_parameter("max_linear_velocity").as_double();
    cfg_.v_min = get_parameter("min_linear_velocity").as_double();
    cfg_.w_max = get_parameter("max_angular_velocity").as_double();
    cfg_.dist_tol = get_parameter("distance_tolerance").as_double();
    cfg_.yaw_tol = get_parameter("yaw_tolerance").as_double();
    cfg_.stop_hold_time = get_parameter("stop_hold_time").as_double();
    cfg_.yaw_stop_hold_time = get_parameter("yaw_stop_hold_time").as_double();
    cfg_.ei_max = get_parameter("linear_integral_max").as_double();
    cfg_.eyaw_i_max = get_parameter("angular_integral_max").as_double();
    cfg_.max_total_time = get_parameter("max_total_time").as_double();
    cfg_.cmd_topic = get_parameter("cmd_vel_topic").as_string();

    mt_cfg_.dt_s = cfg_.dt_s;
    mt_cfg_.Kp_dist = cfg_.Kp;
    mt_cfg_.Kp_yaw = cfg_.KyawP;
    mt_cfg_.v_max = cfg_.v_max;
    mt_cfg_.w_max = cfg_.w_max;
    mt_cfg_.v_min = cfg_.v_min;
    mt_cfg_.dist_tol = cfg_.dist_tol;
    mt_cfg_.yaw_tol = cfg_.yaw_tol;
    mt_cfg_.stop_hold_time = cfg_.stop_hold_time;
    mt_cfg_.max_total_time = cfg_.max_total_time;
    mt_cfg_.cmd_topic = cfg_.cmd_topic;
  }

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  kobuki_ops::OdomCache odom_;
  kobuki_ops::MovePIDConfig cfg_;
  kobuki_ops::MoveToConfig mt_cfg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
