// =============================================================================
// kobuki_operations.hpp — ROS 2 Lifecycle-aware Kobuki base motion primitives
// =============================================================================
#ifndef PATRICK_NAVIGATION__KOBUKI_OPERATIONS_HPP_
#define PATRICK_NAVIGATION__KOBUKI_OPERATIONS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>

#include <atomic>
#include <cmath>
#include <algorithm>
#include <string>
#include <functional>

namespace kobuki_ops
{

// ─── Global done flag ───────────────────────────────────────────────────────
inline std::atomic_bool done_flag{false};
inline void reset_done_flag() { done_flag.store(false); }

// ─── Utility functions ──────────────────────────────────────────────────────
inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

inline double wrap_to_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ─── PID configuration ─────────────────────────────────────────────────────
struct MovePIDConfig
{
  double dt_s = 0.02;

  // Linear distance PID
  double Kp = 1.8;
  double Ki = 0.08;
  double Kd = 0.12;

  // Yaw PID
  double KyawP = 3.0;
  double KyawI = 0.05;
  double KyawD = 0.20;

  // Limits
  double v_max = 0.25;
  double v_min = 0.02;
  double w_max = 1.2;

  // Finish conditions
  double dist_tol = 0.02;
  double stop_hold_time = 0.20;
  double yaw_tol = 0.02;
  double yaw_stop_hold_time = 0.20;

  // Integral anti-windup
  double ei_max = 0.40;
  double eyaw_i_max = 0.50;

  // Safety timeout
  double max_total_time = 20.0;

  std::string cmd_topic = "/diff_drive_controller/cmd_vel_unstamped";
};

struct MoveToConfig
{
  double dt_s = 0.02;
  double Kp_dist = 0.8;
  double Kp_yaw = 2.5;
  double v_max = 0.25;
  double w_max = 1.2;
  double v_min = 0.02;
  double dist_tol = 0.02;
  double yaw_tol = 0.03;
  double face_goal_yaw_tol = 0.15;
  double stop_hold_time = 0.20;
  double max_total_time = 20.0;
  std::string cmd_topic = "/diff_drive_controller/cmd_vel_unstamped";
};

// ─── Odometry cache ────────────────────────────────────────────────────────
struct OdomCache
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool valid = false;

  void update(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    yaw = tf2::getYaw(q);
    valid = true;
  }
};

// ─── Blocking turn-to-yaw ──────────────────────────────────────────────────
inline bool turn_to_yaw_blocking(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double yaw_target,
  const MovePIDConfig & cfg)
{
  reset_done_flag();

  double eyaw_i = 0.0;
  double eyaw_prev = 0.0;
  bool init = false;
  double within_tol_time = 0.0;

  rclcpp::Rate rate(std::max(1.0, 1.0 / std::max(1e-3, cfg.dt_s)));
  auto t0 = node->now();

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (!odom.valid) {
      rate.sleep();
      continue;
    }

    double total_t = (node->now() - t0).seconds();
    if (total_t > cfg.max_total_time) {
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      done_flag.store(false);
      return false;
    }

    double e_yaw = wrap_to_pi(yaw_target - odom.yaw);

    if (!init) {
      init = true;
      eyaw_prev = e_yaw;
    }

    eyaw_i += e_yaw * cfg.dt_s;
    eyaw_i = clamp(eyaw_i, -cfg.eyaw_i_max, cfg.eyaw_i_max);
    double eyd = (e_yaw - eyaw_prev) / std::max(1e-6, cfg.dt_s);
    eyaw_prev = e_yaw;

    double w = cfg.KyawP * e_yaw + cfg.KyawI * eyaw_i + cfg.KyawD * eyd;
    w = clamp(w, -cfg.w_max, cfg.w_max);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = w;
    pub->publish(cmd);

    if (std::fabs(e_yaw) < cfg.yaw_tol) within_tol_time += cfg.dt_s;
    else within_tol_time = 0.0;

    if (within_tol_time >= cfg.yaw_stop_hold_time) {
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      done_flag.store(true);
      return true;
    }

    rate.sleep();
  }

  return false;
}

// ─── Blocking go-distance holding yaw ──────────────────────────────────────
inline bool go_distance_blocking(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double target_dist_signed,
  double yaw_ref,
  const MovePIDConfig & cfg)
{
  reset_done_flag();

  double x0 = odom.x, y0 = odom.y;
  double ei = 0.0, e_prev = 0.0;
  double eyaw_i = 0.0, eyaw_prev = 0.0;
  double within_tol_time = 0.0;

  rclcpp::Rate rate(std::max(1.0, 1.0 / std::max(1e-3, cfg.dt_s)));
  auto t0 = node->now();

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (!odom.valid) { rate.sleep(); continue; }

    double total_t = (node->now() - t0).seconds();
    if (total_t > cfg.max_total_time) {
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      done_flag.store(false);
      return false;
    }

    double dx = odom.x - x0;
    double dy = odom.y - y0;
    double forward = dx * std::cos(yaw_ref) + dy * std::sin(yaw_ref);

    double e_dist = target_dist_signed - forward;
    double e_yaw = wrap_to_pi(yaw_ref - odom.yaw);

    // Distance PID
    ei += e_dist * cfg.dt_s;
    ei = clamp(ei, -cfg.ei_max, cfg.ei_max);
    double ed = (e_dist - e_prev) / std::max(1e-6, cfg.dt_s);
    e_prev = e_dist;
    double v = cfg.Kp * e_dist + cfg.Ki * ei + cfg.Kd * ed;
    v = clamp(v, -cfg.v_max, cfg.v_max);
    if (std::fabs(e_dist) < 0.03 && std::fabs(v) < cfg.v_min)
      v = (v >= 0.0 ? cfg.v_min : -cfg.v_min);

    // Yaw PID
    eyaw_i += e_yaw * cfg.dt_s;
    eyaw_i = clamp(eyaw_i, -cfg.eyaw_i_max, cfg.eyaw_i_max);
    double eyd = (e_yaw - eyaw_prev) / std::max(1e-6, cfg.dt_s);
    eyaw_prev = e_yaw;
    double w = cfg.KyawP * e_yaw + cfg.KyawI * eyaw_i + cfg.KyawD * eyd;
    w = clamp(w, -cfg.w_max, cfg.w_max);

    if (std::fabs(e_dist) < cfg.dist_tol) within_tol_time += cfg.dt_s;
    else within_tol_time = 0.0;

    if (within_tol_time >= cfg.stop_hold_time) {
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      done_flag.store(true);
      return true;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    pub->publish(cmd);

    rate.sleep();
  }

  return false;
}

// ─── Move forward ──────────────────────────────────────────────────────────
inline bool move_forward(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double distance_m,
  const MovePIDConfig & cfg)
{
  if (!odom.valid) return false;
  return go_distance_blocking(node, pub, odom, std::fabs(distance_m), odom.yaw, cfg);
}

// ─── Move backward ────────────────────────────────────────────────────────
inline bool move_backward(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double distance_m,
  const MovePIDConfig & cfg)
{
  if (!odom.valid) return false;
  return go_distance_blocking(node, pub, odom, -std::fabs(distance_m), odom.yaw, cfg);
}

// ─── Move leftward (turn + go + turn back) ─────────────────────────────────
inline bool move_leftward(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double distance_m,
  const MovePIDConfig & cfg,
  double turn_rad = M_PI / 2.0)
{
  if (!odom.valid) return false;
  double yaw_ref = odom.yaw;
  double yaw_goal = wrap_to_pi(yaw_ref + std::fabs(turn_rad));

  if (!turn_to_yaw_blocking(node, pub, odom, yaw_goal, cfg)) return false;
  reset_done_flag();
  if (!go_distance_blocking(node, pub, odom, std::fabs(distance_m), yaw_goal, cfg)) return false;
  reset_done_flag();
  return turn_to_yaw_blocking(node, pub, odom, yaw_ref, cfg);
}

// ─── Move rightward (turn + go + turn back) ────────────────────────────────
inline bool move_rightward(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double distance_m,
  const MovePIDConfig & cfg,
  double turn_rad = M_PI / 2.0)
{
  if (!odom.valid) return false;
  double yaw_ref = odom.yaw;
  double yaw_goal = wrap_to_pi(yaw_ref - std::fabs(turn_rad));

  if (!turn_to_yaw_blocking(node, pub, odom, yaw_goal, cfg)) return false;
  reset_done_flag();
  if (!go_distance_blocking(node, pub, odom, std::fabs(distance_m), yaw_goal, cfg)) return false;
  reset_done_flag();
  return turn_to_yaw_blocking(node, pub, odom, yaw_ref, cfg);
}

// ─── Move to point and yaw (3-phase) ───────────────────────────────────────
inline bool move_to_point_and_yaw(
  rclcpp::Node::SharedPtr node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub,
  OdomCache & odom,
  double x_des, double y_des, double yaw_des,
  const MoveToConfig & cfg)
{
  reset_done_flag();

  rclcpp::Rate rate(std::max(1.0, 1.0 / std::max(1e-3, cfg.dt_s)));
  auto t_start = node->now();
  int phase = 0;
  double within_tol_time = 0.0;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (!odom.valid) { rate.sleep(); continue; }

    double total_t = (node->now() - t_start).seconds();
    if (total_t > cfg.max_total_time) {
      geometry_msgs::msg::Twist stop;
      pub->publish(stop);
      done_flag.store(false);
      return false;
    }

    double dx = x_des - odom.x;
    double dy = y_des - odom.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    double yaw_to_goal = std::atan2(dy, dx);
    double e_face = wrap_to_pi(yaw_to_goal - odom.yaw);
    double e_yaw_final = wrap_to_pi(yaw_des - odom.yaw);

    geometry_msgs::msg::Twist cmd;

    if (phase == 0) {
      if (dist < cfg.dist_tol) { phase = 2; within_tol_time = 0.0; rate.sleep(); continue; }
      double w = clamp(cfg.Kp_yaw * e_face, -cfg.w_max, cfg.w_max);
      cmd.angular.z = w;
      pub->publish(cmd);
      if (std::fabs(e_face) < cfg.face_goal_yaw_tol) { phase = 1; within_tol_time = 0.0; }
    } else if (phase == 1) {
      double v = clamp(cfg.Kp_dist * dist, 0.0, cfg.v_max);
      if (dist > cfg.dist_tol && v < cfg.v_min) v = cfg.v_min;
      double w = clamp(cfg.Kp_yaw * e_face, -cfg.w_max, cfg.w_max);
      if (std::fabs(e_face) > 0.6) v = 0.0;
      cmd.linear.x = v;
      cmd.angular.z = w;
      pub->publish(cmd);
      if (dist < cfg.dist_tol) within_tol_time += cfg.dt_s;
      else within_tol_time = 0.0;
      if (within_tol_time >= cfg.stop_hold_time) { phase = 2; within_tol_time = 0.0; }
    } else if (phase == 2) {
      double w = clamp(cfg.Kp_yaw * e_yaw_final, -cfg.w_max, cfg.w_max);
      cmd.angular.z = w;
      pub->publish(cmd);
      if (std::fabs(e_yaw_final) < cfg.yaw_tol) within_tol_time += cfg.dt_s;
      else within_tol_time = 0.0;
      if (within_tol_time >= cfg.stop_hold_time) {
        geometry_msgs::msg::Twist stop;
        pub->publish(stop);
        done_flag.store(true);
        return true;
      }
    }

    rate.sleep();
  }

  return false;
}

}  // namespace kobuki_ops

#endif  // PATRICK_NAVIGATION__KOBUKI_OPERATIONS_HPP_
