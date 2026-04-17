// Copyright 2026 Patrick Robot Project
// SPDX-License-Identifier: MIT
//
// ArduinoHardwareInterface — ros2_control SystemInterface plugin
//
// Communicates with an Arduino board over serial to control:
//   Joint1        Stepper motor   (via A4988/DRV8825 driver)
//   Joint2, 3     Dynamixel RX-64 (Protocol 1.0, half-duplex UART on Arduino)
//   Joint4, 5R/L  Dynamixel AX-18A (Protocol 1.0, half-duplex UART on Arduino)
//
// Command interface : POSITION  (joint angle, radians)
// State  interfaces : position, velocity, effort
// GPIO    state     : speed_scaling/speed_scaling_factor  (for ur_controllers
//                     ScaledJointTrajectoryController)
//
// Joint5L is declared as a URDF mimic of Joint5R and therefore exports NO
// command interface; the controller sends one gripper command which this
// interface replicates to both physical fingers internally.
//
// Serial protocol (simple text, newline-delimited):
//   PC → Arduino:  "P <j> <pos_rad>\n"           (Position command)
//   PC → Arduino:  "R\n"                          (Request all states)
//   Arduino → PC:  "S <j> <pos> <vel> <eff>\n"    (per commanded joint)
//   Arduino → PC:  "SS <factor>\n"                (speed scaling in [0,1])

#ifndef PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_
#define PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace patrick_manipulation
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool open_serial_port();
  void close_serial_port();
  bool send_command(const std::string & cmd);
  std::string read_line();

  // Configuration
  std::string port_name_;
  int baud_rate_{115200};
  int serial_fd_{-1};

  // Joint state storage (indexed by joint order in HardwareInfo)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Joint position commands (same indexing)
  std::vector<double> hw_cmd_positions_;

  // Command-interface presence flag (false for mimic joints)
  std::vector<bool> has_command_;

  // Mimic descriptor: mimic_source_[i] >= 0 → joint i follows that source
  std::vector<int> mimic_source_;
  std::vector<double> mimic_multiplier_;
  std::vector<double> mimic_offset_;

  // Speed-scaling GPIO state (consumed by ScaledJointTrajectoryController)
  double speed_scaling_factor_{1.0};
};

}  // namespace patrick_manipulation

#endif  // PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_
// Copyright 2026 Patrick Robot Project
// SPDX-License-Identifier: MIT
//
// ArduinoHardwareInterface — ros2_control SystemInterface plugin
//
// Communicates with an Arduino board over serial to control:
//   Joint1        Stepper motor   (via A4988/DRV8825 driver)
//   Joint2, 3     Dynamixel RX-64 (Protocol 1.0, half-duplex UART on Arduino)
//   Joint4, 5R/L  Dynamixel AX-18A (Protocol 1.0, half-duplex UART on Arduino)
//
// Serial protocol (simple text, newline-delimited):
//   PC → Arduino:  "W <j> <pos> <eff>\n"   (Write command for joint j)
//   Arduino → PC:  "S <j> <pos> <vel>\n"   (State feedback for joint j)
//   PC → Arduino:  "R\n"                    (Request all joint states)

#ifndef PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_
#define PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace patrick_manipulation
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  // ── Lifecycle callbacks ───────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ──────────────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Real-time control loop ────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port helpers
  bool open_serial_port();
  void close_serial_port();
  bool send_command(const std::string & cmd);
  std::string read_line();

  // Configuration
  std::string port_name_;
  int baud_rate_{115200};
  int serial_fd_{-1};

  // Joint state storage  (indexed by joint order in HardwareInfo)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Joint command storage
  std::vector<double> hw_cmd_efforts_;
};

}  // namespace patrick_manipulation

#endif  // PATRICK_MANIPULATION__ARDUINO_HARDWARE_INTERFACE_HPP_
