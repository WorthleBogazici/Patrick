// =============================================================================
// manipulation_node.cpp — Lifecycle node for arm manipulation (ROS 2)
// =============================================================================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "patrick_manipulation/operations.hpp"
#include "patrick_manipulation/states.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using FollowJT = control_msgs::action::FollowJointTrajectory;

class ManipulationNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ManipulationNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("manipulation_node", opts)
  {
    // Declare parameters
    declare_parameter("use_trajectory_controller", true);
    declare_parameter("arm_controller_action", "/arm_controller/follow_joint_trajectory");
    declare_parameter("gripper_controller_topic", "/gripper_controller/commands");
    declare_parameter("wait_subscribers_s", 5.0);

    // Arm DH params
    declare_parameter("arm.d1", 0.08327);
    declare_parameter("arm.d2", 0.08918);
    declare_parameter("arm.d3", 0.03320);
    declare_parameter("arm.a2", 0.18000);
    declare_parameter("arm.a3", 0.22010);

    // Home angles
    declare_parameter("home.theta1", M_PI / 4.0);
    declare_parameter("home.theta2", 0.0);
    declare_parameter("home.theta3", M_PI / 2.0);

    // Velocities
    declare_parameter("homing_velocity.joint1", 0.6);
    declare_parameter("homing_velocity.joint2", 0.6);
    declare_parameter("homing_velocity.joint3", 0.6);

    // Gripper
    declare_parameter("gripper.open_angle", 0.194429679);
    declare_parameter("gripper.close_angle", -0.194429679);

    // Wrist
    declare_parameter("wrist.horizontal", 0.0);
    declare_parameter("wrist.vertical", 1.57);

    // Motion
    declare_parameter("cmd_rate_hz", 50.0);
    declare_parameter("hold_sec_default", 2.0);
    declare_parameter("ee_tol_m", 0.01);
    declare_parameter("ik_move_sec", 2.0);

    // Pure pursuit
    declare_parameter("pp_lookahead_u", 0.06);
    declare_parameter("pp_err_slow_rad", 0.15);
    declare_parameter("pp_alpha_min", 0.02);
    declare_parameter("pp_q_finish_tol", 0.03);
    declare_parameter("pp_max_total_sec", 10.0);

    // Shake
    declare_parameter("shake_frequency_hz", 10.0);
    declare_parameter("shake_j1_position", 0.78);
    declare_parameter("shake_j2_position", 0.0);
    declare_parameter("shake_j3_a_position", 0.5);
    declare_parameter("shake_j3_b_position", -0.5);

    // Knock
    declare_parameter("knock_offset", 0.08);
    declare_parameter("knock_period", 0.2);

    // Push/pull
    declare_parameter("push_distance", 0.07);
    declare_parameter("pull_distance", 0.07);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring ManipulationNode");
    load_params();

    arm_action_client_ = rclcpp_action::create_client<FollowJT>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      get_parameter("arm_controller_action").as_string());

    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      get_parameter("gripper_controller_topic").as_string(), 10);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { js_cache_.cb(msg); });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "ManipulationNode active");
    gripper_pub_->on_activate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    gripper_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    arm_action_client_.reset();
    gripper_pub_.reset();
    js_sub_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  // ── Public API ──────────────────────────────────────────────────────────
  bool go_home()
  {
    auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
    return patrick_manip::get_home(node_ptr, arm_action_client_, js_cache_, cfg_);
  }

  bool take_object(double x, double y, double z, bool wrist_vert = false)
  {
    auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
    return patrick_manip::take(node_ptr, arm_action_client_, gripper_pub_,
      js_cache_, cfg_, x, y, z, wrist_vert);
  }

  bool move_to(double x, double y, double z)
  {
    auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
    return patrick_manip::move_arm(node_ptr, arm_action_client_, js_cache_, cfg_, x, y, z);
  }

  bool put_object(double x, double y, double z)
  {
    auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
    return patrick_manip::put(node_ptr, arm_action_client_, gripper_pub_, js_cache_, cfg_, x, y, z);
  }

  const patrick_manip::ArmConfig & config() const { return cfg_; }

private:
  void load_params()
  {
    cfg_.dh.d1 = get_parameter("arm.d1").as_double();
    cfg_.dh.d2 = get_parameter("arm.d2").as_double();
    cfg_.dh.d3 = get_parameter("arm.d3").as_double();
    cfg_.dh.a2 = get_parameter("arm.a2").as_double();
    cfg_.dh.a3 = get_parameter("arm.a3").as_double();

    cfg_.theta1_home = get_parameter("home.theta1").as_double();
    cfg_.theta2_home = get_parameter("home.theta2").as_double();
    cfg_.theta3_home = get_parameter("home.theta3").as_double();

    cfg_.theta1_vel = get_parameter("homing_velocity.joint1").as_double();
    cfg_.theta2_vel = get_parameter("homing_velocity.joint2").as_double();
    cfg_.theta3_vel = get_parameter("homing_velocity.joint3").as_double();

    cfg_.gripper_open = get_parameter("gripper.open_angle").as_double();
    cfg_.gripper_close = get_parameter("gripper.close_angle").as_double();

    cfg_.wrist_horizontal = get_parameter("wrist.horizontal").as_double();
    cfg_.wrist_vertical = get_parameter("wrist.vertical").as_double();

    cfg_.cmd_rate_hz = get_parameter("cmd_rate_hz").as_double();
    cfg_.hold_sec_default = get_parameter("hold_sec_default").as_double();
    cfg_.ee_tol_m = get_parameter("ee_tol_m").as_double();
    cfg_.ik_move_sec = get_parameter("ik_move_sec").as_double();

    cfg_.pp_lookahead_u = get_parameter("pp_lookahead_u").as_double();
    cfg_.pp_err_slow_rad = get_parameter("pp_err_slow_rad").as_double();
    cfg_.pp_alpha_min = get_parameter("pp_alpha_min").as_double();
    cfg_.pp_q_finish_tol = get_parameter("pp_q_finish_tol").as_double();
    cfg_.pp_max_total_sec = get_parameter("pp_max_total_sec").as_double();
  }

  rclcpp_action::Client<FollowJT>::SharedPtr arm_action_client_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  patrick_manip::JointStateCache js_cache_;
  patrick_manip::ArmConfig cfg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulationNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
