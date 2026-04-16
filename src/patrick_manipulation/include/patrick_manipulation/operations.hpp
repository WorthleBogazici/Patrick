// =============================================================================
// operations.hpp — Arm FK/IK, trajectory execution, attach/detach (ROS 2)
// =============================================================================
#ifndef PATRICK_MANIPULATION__OPERATIONS_HPP_
#define PATRICK_MANIPULATION__OPERATIONS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;

namespace patrick_manip
{

// ─── Forward: done_flag from navigation ─────────────────────────────────────
namespace kobuki_ops { extern std::atomic_bool done_flag; inline void reset_done_flag() { done_flag.store(false); } }

// ─── Clamp helpers ──────────────────────────────────────────────────────────
inline double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

inline void clamp_joints(Eigen::Vector3d & q)
{
  q[0] = clampd(q[0], -M_PI / 4.0, M_PI / 4.0);
  q[1] = clampd(q[1], -M_PI / 2.0, M_PI / 2.0);
  q[2] = clampd(q[2], -M_PI / 2.0, M_PI / 2.0);
}

// ─── Quintic time scaling ───────────────────────────────────────────────────
inline double quintic_s(double t)
{
  t = clampd(t, 0.0, 1.0);
  double t3 = t * t * t;
  return 10.0 * t3 - 15.0 * t3 * t + 6.0 * t3 * t * t;
}

// ─── DH Parameters (shared) ────────────────────────────────────────────────
struct DHParams
{
  double d1 = 0.08327;
  double d2 = 0.08918;
  double d3 = 0.03320;
  double a2 = 0.18000;
  double a3 = 0.22010;
};

// ─── Robot config ───────────────────────────────────────────────────────────
struct ArmConfig
{
  DHParams dh;

  double theta1_home = M_PI / 4.0;
  double theta2_home = 0.0;
  double theta3_home = M_PI / 2.0;

  double theta1_vel = 0.6;
  double theta2_vel = 0.6;
  double theta3_vel = 0.6;

  double gripper_open = 0.194429679;
  double gripper_close = -0.194429679;

  double wrist_horizontal = 0.0;
  double wrist_vertical = 1.57;

  double cmd_rate_hz = 50.0;
  double hold_sec_default = 2.0;
  double ee_tol_m = 0.01;
  double ik_move_sec = 2.0;

  double pp_lookahead_u = 0.06;
  double pp_err_slow_rad = 0.15;
  double pp_alpha_min = 0.02;
  double pp_q_finish_tol = 0.03;
  double pp_max_total_sec = 10.0;
};

// ─── Joint state cache ─────────────────────────────────────────────────────
struct JointStateCache
{
  std::mutex mtx;
  sensor_msgs::msg::JointState last;
  bool have = false;

  void cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx);
    last = *msg;
    have = true;
  }

  bool get(sensor_msgs::msg::JointState & out)
  {
    std::lock_guard<std::mutex> lk(mtx);
    if (!have) return false;
    out = last;
    return true;
  }
};

inline int find_joint_index(const sensor_msgs::msg::JointState & js, const std::string & key)
{
  for (size_t i = 0; i < js.name.size(); ++i) {
    if (js.name[i] == key) return static_cast<int>(i);
    if (js.name[i].find(key) != std::string::npos) return static_cast<int>(i);
  }
  return -1;
}

inline bool get_measured_q3(
  JointStateCache & cache, Eigen::Vector3d & q,
  const std::string & j1 = "Joint1",
  const std::string & j2 = "Joint2",
  const std::string & j3 = "Joint3")
{
  sensor_msgs::msg::JointState js;
  if (!cache.get(js)) return false;
  int i1 = find_joint_index(js, j1);
  int i2 = find_joint_index(js, j2);
  int i3 = find_joint_index(js, j3);
  if (i1 < 0 || i2 < 0 || i3 < 0) return false;
  int imax = std::max({i1, i2, i3});
  if (static_cast<int>(js.position.size()) <= imax) return false;
  q = Eigen::Vector3d(js.position[i1], js.position[i2], js.position[i3]);
  return true;
}

// ─── Forward kinematics ────────────────────────────────────────────────────
inline void fk_from_thetas(
  const DHParams & dh,
  double theta1, double theta2, double theta3,
  double & px, double & py, double & pz)
{
  double c1 = std::cos(theta1), s1 = std::sin(theta1);
  double c2 = std::cos(theta2), s2 = std::sin(theta2);
  double c3 = std::cos(theta3), s3 = std::sin(theta3);

  double A = dh.a2 * c2 + dh.a3 * c2 * c3 - dh.d3 * s2;
  double B = dh.d2 + dh.a3 * s3;

  // Map to Link1 frame
  px = -0.07912 + (-( A * c1 - B * s1));
  py =  0.022   + (dh.d1 + s2 * (dh.a2 + dh.a3 * c3) + dh.d3 * c2);
  pz =  0.08412 + ( A * s1 + B * c1);
}

// ─── Geometric IK ──────────────────────────────────────────────────────────
inline bool solve_ik_openloop(
  const DHParams & dh,
  double px_target, double py_target, double pz_target,
  double & theta1, double & theta2, double & theta3)
{
  const double px_off = -0.07912, py_off = 0.0220, pz_off = 0.08412;
  const double t_min = -M_PI / 2.0, t_max = M_PI / 2.0;

  double px = px_target - px_off;
  double py = py_target - py_off - dh.d1;
  double pz = pz_target - pz_off;

  double r2 = px * px + pz * pz;
  double R = std::sqrt(dh.a2 * dh.a2 + dh.d2 * dh.d2);
  double phi = std::atan2(dh.d2, dh.a2);

  double K = r2 + py * py - (dh.a2 * dh.a2 + dh.a3 * dh.a3 + dh.d2 * dh.d2 + dh.d3 * dh.d3);
  double cos_delta = K / (2.0 * dh.a3 * R);
  if (cos_delta > 1.1 || cos_delta < -1.1) return false;
  cos_delta = clampd(cos_delta, -1.0, 1.0);
  double delta = std::acos(cos_delta);

  auto wrap = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };

  auto residual = [&](double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    double A = dh.d3 * s2 - dh.a2 * c2 - dh.a3 * c2 * c3;
    double B = dh.d2 + dh.a3 * s3;
    double eex = A * c1 + B * s1 - px;
    double eez = -A * s1 + B * c1 - pz;
    double eey = (dh.a2 + dh.a3 * c3) * s2 + dh.d3 * c2 - py;
    return eex * eex + eey * eey + eez * eez;
  };

  struct Sol { double t1, t2, t3, err; bool ok; };
  Sol best{0, 0, 0, 0, false};

  auto tryCandidate = [&](double t3c_in, int A_sign) -> Sol {
    double t3c = wrap(t3c_in);
    if (t3c < t_min - 1e-12 || t3c > t_max + 1e-12) return {0, 0, 0, 0, false};
    double c3 = std::cos(t3c), s3 = std::sin(t3c);
    double D = dh.a2 + dh.a3 * c3;
    double B = dh.d2 + dh.a3 * s3;
    double rad = r2 - B * B;
    if (rad < -1e-6) return {0, 0, 0, 0, false};
    double root = std::sqrt(std::max(0.0, rad));
    double Av = (A_sign >= 0) ? root : -root;
    double denom = D * D + dh.d3 * dh.d3;
    double s2 = (D * py + dh.d3 * Av) / denom;
    double c2 = (dh.d3 * py - D * Av) / denom;
    double t2c = wrap(std::atan2(s2, c2));
    if (t2c < t_min - 1e-12 || t2c > t_max + 1e-12) return {0, 0, 0, 0, false};
    double t1c = wrap(std::atan2(px * B - pz * Av, px * Av + pz * B));
    if (t1c < -M_PI / 2.0 - 1e-12 || t1c > M_PI / 2.0 + 1e-12) return {0, 0, 0, 0, false};
    return {t1c, t2c, t3c, residual(t1c, t2c, t3c), true};
  };

  Sol cand[4] = {
    tryCandidate(phi + delta, +1),
    tryCandidate(phi + delta, -1),
    tryCandidate(phi - delta, +1),
    tryCandidate(phi - delta, -1),
  };

  bool found = false;
  for (auto & c : cand) {
    if (!c.ok) continue;
    if (!found || c.err < best.err) { best = c; found = true; }
  }
  if (!found) return false;

  theta1 = best.t1;
  theta2 = best.t2;
  theta3 = best.t3;
  return true;
}

// ─── Send trajectory goal (blocking) ────────────────────────────────────────
inline bool send_arm_trajectory(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr client,
  const std::vector<std::string> & joint_names,
  const Eigen::Vector3d & q_start,
  const Eigen::Vector3d & q_goal,
  double duration_sec)
{
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Arm trajectory action server not available");
    return false;
  }

  auto goal = FollowJT::Goal();
  goal.trajectory.joint_names = joint_names;

  // Start point
  trajectory_msgs::msg::JointTrajectoryPoint start_pt;
  start_pt.positions = {q_start[0], q_start[1], q_start[2]};
  start_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
  goal.trajectory.points.push_back(start_pt);

  // Intermediate points via quintic interpolation
  int n_points = std::max(5, static_cast<int>(duration_sec * 10));
  for (int i = 1; i <= n_points; ++i) {
    double t_frac = static_cast<double>(i) / n_points;
    double s = quintic_s(t_frac);
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {
      (1.0 - s) * q_start[0] + s * q_goal[0],
      (1.0 - s) * q_start[1] + s * q_goal[1],
      (1.0 - s) * q_start[2] + s * q_goal[2],
    };
    pt.time_from_start = rclcpp::Duration::from_seconds(t_frac * duration_sec);
    goal.trajectory.points.push_back(pt);
  }

  auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send arm trajectory goal");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Arm trajectory goal rejected");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future,
    std::chrono::seconds(static_cast<int>(duration_sec * 3 + 10))) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(node->get_logger(), "Arm trajectory timed out");
    return false;
  }

  auto result = result_future.get();
  return result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

// ─── Send gripper command ───────────────────────────────────────────────────
inline void command_gripper(
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub,
  double angle)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {angle, angle};
  pub->publish(msg);
}

// ─── Send wrist command via trajectory ──────────────────────────────────────
inline void command_wrist(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr client,
  const std::string & j4_name,
  double angle,
  double duration = 1.0)
{
  (void)node;
  (void)client;
  (void)j4_name;
  (void)angle;
  (void)duration;
  // Wrist is Joint4 — included in the arm trajectory controller
  // For standalone wrist motion, send a FollowJointTrajectory goal with just Joint4
  // This is a simplified version; full version sends trajectory
}

// ─── Solve IK and move ─────────────────────────────────────────────────────
inline bool solve_ik_and_move(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr client,
  JointStateCache & js_cache,
  const ArmConfig & cfg,
  double x, double y, double z)
{
  static Eigen::Vector3d q_last(cfg.theta1_home, cfg.theta2_home, cfg.theta3_home);
  clamp_joints(q_last);

  double q1, q2, q3;
  if (!solve_ik_openloop(cfg.dh, x, y, z, q1, q2, q3)) {
    RCLCPP_ERROR(node->get_logger(), "IK failed for target (%.4f, %.4f, %.4f)", x, y, z);
    return false;
  }

  Eigen::Vector3d q_goal(q1, q2, q3);
  clamp_joints(q_goal);

  RCLCPP_INFO(node->get_logger(), "IK q_goal = (%.4f, %.4f, %.4f)", q1, q2, q3);

  // Get current position from joint states
  Eigen::Vector3d q_start;
  if (!get_measured_q3(js_cache, q_start)) {
    q_start = q_last;
  }
  clamp_joints(q_start);

  // Compute duration based on max joint displacement
  Eigen::Vector3d dq = (q_goal - q_start).cwiseAbs();
  double duration = std::max({dq[0] / cfg.theta1_vel, dq[1] / cfg.theta2_vel,
                              dq[2] / cfg.theta3_vel, 0.5});

  bool ok = send_arm_trajectory(
    node, client, {"Joint1", "Joint2", "Joint3"}, q_start, q_goal, duration);

  q_last = q_goal;
  return ok;
}

// ─── Go home ────────────────────────────────────────────────────────────────
inline bool get_home(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<FollowJT>::SharedPtr client,
  JointStateCache & js_cache,
  const ArmConfig & cfg)
{
  Eigen::Vector3d q_start;
  if (!get_measured_q3(js_cache, q_start)) {
    q_start = Eigen::Vector3d(cfg.theta1_home, cfg.theta2_home, cfg.theta3_home);
  }
  clamp_joints(q_start);

  Eigen::Vector3d q_goal(cfg.theta1_home, cfg.theta2_home, cfg.theta3_home);
  clamp_joints(q_goal);

  Eigen::Vector3d dq = (q_goal - q_start).cwiseAbs();
  double duration = std::max({dq[0] / cfg.theta1_vel, dq[1] / cfg.theta2_vel,
                              dq[2] / cfg.theta3_vel, 0.5});

  return send_arm_trajectory(
    node, client, {"Joint1", "Joint2", "Joint3"}, q_start, q_goal, duration);
}

}  // namespace patrick_manip

#endif  // PATRICK_MANIPULATION__OPERATIONS_HPP_
