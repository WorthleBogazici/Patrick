// =============================================================================
// states.hpp — High-level manipulation states (take, put, move, shake, etc.)
// =============================================================================
#ifndef PATRICK_MANIPULATION__STATES_HPP_
#define PATRICK_MANIPULATION__STATES_HPP_

#include "patrick_manipulation/operations.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>
#include <thread>
#include <atomic>
#include <string>

namespace patrick_manip
{

// ─── State helpers ──────────────────────────────────────────────────────────
namespace detail
{
inline std::atomic_bool & done_flag()
{
  // Shared with navigation
  return kobuki_ops::done_flag;
}

inline void state_begin()  { done_flag().store(false); }
inline void state_success() { done_flag().store(true); }
inline void state_fail()    { done_flag().store(false); }
}  // namespace detail

// ─── Timings ────────────────────────────────────────────────────────────────
struct TakeTimings
{
  double after_open_sec = 2.0;
  double after_wrist_sec = 2.0;
  double after_ik_sec = 1.0;
  double after_attach_init_sec = 0.0;
  double after_close_sec = 0.0;
};

// ─── Shake parameters ──────────────────────────────────────────────────────
struct ShakeParams
{
  double frequency_hz = 10.0;
  double j1_position = 0.78;
  double j2_position = 0.0;
  double j3_a_position = 0.5;
  double j3_b_position = -0.5;
};

// ─── Knock parameters ──────────────────────────────────────────────────────
struct KnockParams
{
  double offset = 0.08;
  double period = 0.2;
};

// ─── Helper: sleep ──────────────────────────────────────────────────────────
inline void sleep_sec(double sec)
{
  if (sec > 0.01) {
    std::this_thread::sleep_for(
      std::chrono::duration<double>(sec));
  }
}

// ─── take() ─────────────────────────────────────────────────────────────────
// Moves arm to object, attaches (opens gripper, IK to object, closes gripper)
inline bool take(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client,
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double obj_x, double obj_y, double obj_z,
  bool wrist_vert = false,
  const TakeTimings & T = TakeTimings())
{
  detail::state_begin();

  // Open gripper
  command_gripper(gripper_pub, cfg.gripper_open);
  sleep_sec(T.after_open_sec);

  // (Wrist handled by trajectory controller — Joint4 set separately if needed)
  (void)wrist_vert;
  sleep_sec(T.after_wrist_sec);

  // IK to object
  if (!solve_ik_and_move(node, arm_client, js_cache, cfg, obj_x, obj_y, obj_z)) {
    detail::state_fail();
    return false;
  }
  sleep_sec(T.after_ik_sec);

  // Close gripper (simulates attach)
  command_gripper(gripper_pub, cfg.gripper_close);
  sleep_sec(T.after_close_sec);

  detail::state_success();
  return true;
}

// ─── move() ─────────────────────────────────────────────────────────────────
inline bool move_arm(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double x, double y, double z)
{
  detail::state_begin();

  if (!solve_ik_and_move(node, arm_client, js_cache, cfg, x, y, z)) {
    detail::state_fail();
    return false;
  }

  detail::state_success();
  return true;
}

// ─── put() ──────────────────────────────────────────────────────────────────
inline bool put(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client,
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double x, double y, double z,
  const TakeTimings & T = TakeTimings())
{
  detail::state_begin();

  if (!solve_ik_and_move(node, arm_client, js_cache, cfg, x, y, z)) {
    detail::state_fail();
    return false;
  }
  sleep_sec(T.after_ik_sec);

  command_gripper(gripper_pub, cfg.gripper_open);
  sleep_sec(T.after_open_sec);

  detail::state_success();
  return true;
}

// ─── shake() ────────────────────────────────────────────────────────────────
inline bool shake(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double shake_time_sec,
  const ShakeParams & sp = ShakeParams())
{
  detail::state_begin();
  (void)js_cache;

  double hz = std::max(0.5, sp.frequency_hz);
  int n_toggles = static_cast<int>(shake_time_sec * hz);
  double gap = 1.0 / hz;

  for (int i = 0; i < n_toggles; ++i) {
    double j3 = (i % 2 == 0) ? sp.j3_a_position : sp.j3_b_position;
    Eigen::Vector3d q(sp.j1_position, sp.j2_position, j3);
    Eigen::Vector3d q_start = q;
    q_start[2] = (i % 2 == 0) ? sp.j3_b_position : sp.j3_a_position;

    send_arm_trajectory(node, arm_client, {"Joint1", "Joint2", "Joint3"}, q_start, q, gap);
  }

  detail::state_success();
  return true;
}

// ─── knock() ────────────────────────────────────────────────────────────────
inline bool knock(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr arm_client,
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double obj_x, double obj_y, double obj_z,
  const KnockParams & kp = KnockParams())
{
  detail::state_begin();

  command_gripper(gripper_pub, cfg.gripper_close);
  sleep_sec(0.5);

  // Move above object
  if (!solve_ik_and_move(node, arm_client, js_cache, cfg,
    obj_x, obj_y + kp.offset, obj_z))
  {
    detail::state_fail();
    return false;
  }

  detail::state_success();
  return true;
}

}  // namespace patrick_manip

#endif  // PATRICK_MANIPULATION__STATES_HPP_
