// functionality_check.cpp — Sequential ±30° arm joint test (loop)
//
// Sends relative +30° then −30° to each joint in order:
//   Joint1 → Joint2 → Joint3 → Joint4 → Joint5R → Joint5L
//
// All 6 joints are managed by a single arm_controller
// (JointTrajectoryController with effort command interface and internal PID).
// Every move sends a trajectory that moves ONE target joint while holding
// all others at their current positions.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <thread>
#include <chrono>
#include <atomic>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class FunctionalityCheck : public rclcpp::Node
{
public:
  FunctionalityCheck()
  : Node("functionality_check")
  {
    all_joints_ = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5R", "Joint5L"};

    for (const auto & name : all_joints_) {
      current_positions_[name] = 0.0;
    }

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pos_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (current_positions_.count(msg->name[i])) {
            current_positions_[msg->name[i]] = msg->position[i];
          }
        }
        got_joint_states_ = true;
      });

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Waiting for joint states and action server...");
  }

  void run_sequence()
  {
    // Wait for joint states
    while (rclcpp::ok() && !got_joint_states_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(this->get_logger(), "Got joint states.");

    // Wait for action server
    RCLCPP_INFO(this->get_logger(), "Waiting for arm_controller action server...");
    while (rclcpp::ok() &&
           !action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_INFO(this->get_logger(), "  Still waiting...");
    }
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(this->get_logger(), "Connected. Starting functionality check loop.");

    // Initial hold on all joints
    send_hold(HOLD_DURATION_SEC);

    int cycle = 0;
    while (rclcpp::ok()) {
      cycle++;
      RCLCPP_INFO(this->get_logger(), "=== Cycle %d: +30 deg sweep ===", cycle);
      for (const auto & joint : all_joints_) {
        if (!rclcpp::ok()) return;
        double angle = (joint == "Joint5R" || joint == "Joint5L") ? DEG22 : DEG30;
        move_single_joint(joint, angle);
      }

      send_hold(HOLD_DURATION_SEC);

      RCLCPP_INFO(this->get_logger(), "=== Cycle %d: -30 deg sweep ===", cycle);
      for (const auto & joint : all_joints_) {
        if (!rclcpp::ok()) return;
        double angle = (joint == "Joint5R" || joint == "Joint5L") ? DEG22 : DEG30;
        move_single_joint(joint, -angle);
      }

      send_hold(HOLD_DURATION_SEC);
    }
  }

private:
  static constexpr double DEG30 = 30.0 * M_PI / 180.0;
  static constexpr double DEG22 = 22.0 * M_PI / 180.0;  // within ±0.393 rad gripper limits
  static constexpr double MOVE_DURATION_SEC = 2.0;
  static constexpr double HOLD_DURATION_SEC = 5.0;

  std::vector<std::string> all_joints_;

  std::map<std::string, double> current_positions_;
  std::mutex pos_mutex_;
  std::atomic<bool> got_joint_states_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  std::map<std::string, double> get_positions()
  {
    std::lock_guard<std::mutex> lock(pos_mutex_);
    return current_positions_;
  }

  // Send a hold-position trajectory for all joints (fire and forget)
  void send_hold(double duration_sec)
  {
    auto positions = get_positions();
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & j : all_joints_) {
      point.positions.push_back(positions[j]);
      point.velocities.push_back(0.0);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal_msg.trajectory.points.push_back(point);

    action_client_->async_send_goal(goal_msg);
  }

  // Move a single joint by delta while holding all others
  void move_single_joint(const std::string & target_joint, double delta)
  {
    auto positions = get_positions();

    RCLCPP_INFO(this->get_logger(), "  Moving %s by %+.1f deg (holding all others)",
                target_joint.c_str(), delta * 180.0 / M_PI);

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & j : all_joints_) {
      double pos = positions[j];
      if (j == target_joint) {
        pos += delta;
      }
      point.positions.push_back(pos);
      point.velocities.push_back(0.0);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(MOVE_DURATION_SEC);
    goal_msg.trajectory.points.push_back(point);

    auto goal_future = action_client_->async_send_goal(goal_msg);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Timeout sending goal for %s", target_joint.c_str());
      return;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected for %s", target_joint.c_str());
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(MOVE_DURATION_SEC + 5.0));
    result_future.wait_for(timeout);

    RCLCPP_INFO(this->get_logger(), "  %s done.", target_joint.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FunctionalityCheck>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  node->run_sequence();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
