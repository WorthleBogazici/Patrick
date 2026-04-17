// Copyright 2026 Patrick Robot Project
// SPDX-License-Identifier: MIT

#include "patrick_manipulation/arduino_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace patrick_manipulation
{

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_init
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters.count("port_name")
    ? info_.hardware_parameters.at("port_name") : "/dev/ttyACM0";
  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;

  const auto num_joints = info_.joints.size();
  hw_positions_.assign(num_joints, 0.0);
  hw_velocities_.assign(num_joints, 0.0);
  hw_efforts_.assign(num_joints, 0.0);
  hw_cmd_positions_.assign(num_joints, std::numeric_limits<double>::quiet_NaN());
  has_command_.assign(num_joints, false);
  mimic_source_.assign(num_joints, -1);
  mimic_multiplier_.assign(num_joints, 1.0);
  mimic_offset_.assign(num_joints, 0.0);

  // Build mimic map from <param name="mimic">... in ros2_control block
  for (size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];
    has_command_[i] = !joint.command_interfaces.empty();

    auto it = joint.parameters.find("mimic");
    if (it != joint.parameters.end()) {
      const std::string & src_name = it->second;
      int src_idx = -1;
      for (size_t k = 0; k < num_joints; ++k) {
        if (info_.joints[k].name == src_name) { src_idx = static_cast<int>(k); break; }
      }
      if (src_idx < 0) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArduinoHardwareInterface"),
          "Joint '%s' mimics unknown joint '%s'",
          joint.name.c_str(), src_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      mimic_source_[i] = src_idx;
      auto mit = joint.parameters.find("multiplier");
      if (mit != joint.parameters.end()) {
        mimic_multiplier_[i] = std::stod(mit->second);
      }
      auto oit = joint.parameters.find("offset");
      if (oit != joint.parameters.end()) {
        mimic_offset_[i] = std::stod(oit->second);
      }
    }
  }

  // Validate: non-mimic joints with a command must have exactly 1 command interface
  for (size_t i = 0; i < num_joints; ++i) {
    const auto & joint = info_.joints[i];
    if (mimic_source_[i] >= 0) {
      if (!joint.command_interfaces.empty()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ArduinoHardwareInterface"),
          "Mimic joint '%s' must not declare a command interface",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      continue;
    }
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArduinoHardwareInterface"),
        "Joint '%s' must expose exactly 1 'position' command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"),
    "Initialized with %zu joints, port=%s, baud=%d",
    num_joints, port_name_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_configure  — open serial port
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial_port()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArduinoHardwareInterface"),
      "Failed to open serial port '%s'", port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"),
    "Serial port '%s' opened at %d baud", port_name_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_activate  — enable motors, seed command = current state
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_command("E\n");  // Enable motors

  send_command("R\n");
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    auto line = read_line();
    if (line.empty()) { continue; }
    int idx = 0;
    double pos = 0.0, vel = 0.0, eff = 0.0;
    std::istringstream iss(line);
    std::string tag;
    if (iss >> tag >> idx >> pos >> vel >> eff) {
      if (tag == "S" && idx >= 0 && idx < static_cast<int>(hw_positions_.size())) {
        hw_positions_[idx] = pos;
        hw_velocities_[idx] = vel;
        hw_efforts_[idx] = eff;
      }
    }
  }

  // Seed position commands to the current read position so the controller
  // doesn't snap the robot on first write().
  for (size_t i = 0; i < hw_cmd_positions_.size(); ++i) {
    if (has_command_[i]) {
      hw_cmd_positions_[i] = hw_positions_[i];
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"), "Motors enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_command("D\n");
  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"), "Motors disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Interface export
// ═══════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }

  // Export GPIO speed_scaling/speed_scaling_factor for ur_controllers
  state_interfaces.emplace_back(
    "speed_scaling", "speed_scaling_factor", &speed_scaling_factor_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (!has_command_[i]) { continue; }  // Skip mimic joints
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_cmd_positions_[i]);
  }
  return command_interfaces;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Real-time loop: read
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  send_command("R\n");

  // Drain all available lines (S lines for joints, SS line for speed scaling).
  // Number of lines is bounded by number of commanded joints + 1.
  const size_t max_lines = hw_positions_.size() + 1;
  for (size_t k = 0; k < max_lines; ++k) {
    auto line = read_line();
    if (line.empty()) { break; }

    std::istringstream iss(line);
    std::string tag;
    iss >> tag;

    if (tag == "S") {
      int idx = 0;
      double pos = 0.0, vel = 0.0, eff = 0.0;
      if (iss >> idx >> pos >> vel >> eff) {
        if (idx >= 0 && idx < static_cast<int>(hw_positions_.size())) {
          hw_positions_[idx] = pos;
          hw_velocities_[idx] = vel;
          hw_efforts_[idx] = eff;
        }
      }
    } else if (tag == "SS") {
      double factor = 1.0;
      if (iss >> factor) {
        if (factor < 0.0) { factor = 0.0; }
        if (factor > 1.0) { factor = 1.0; }
        speed_scaling_factor_ = factor;
      }
    }
  }

  // Derive mimic joint states from their source (some firmware won't report
  // the mimic joint itself, so mirror it here for consistent state broadcast).
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    const int src = mimic_source_[i];
    if (src >= 0) {
      hw_positions_[i]  = mimic_multiplier_[i] * hw_positions_[src] + mimic_offset_[i];
      hw_velocities_[i] = mimic_multiplier_[i] * hw_velocities_[src];
    }
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Real-time loop: write
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < hw_cmd_positions_.size(); ++i) {
    if (!has_command_[i]) { continue; }
    const double cmd = hw_cmd_positions_[i];
    if (std::isnan(cmd)) { continue; }

    std::ostringstream oss;
    oss << "P " << i << " " << cmd << "\n";
    send_command(oss.str());
  }

  // Replicate Joint5R → Joint5L physically (firmware drives two servos on one
  // logical command; we also emit an explicit P command for the mimic index
  // in case firmware expects per-servo commands).
  for (size_t i = 0; i < hw_cmd_positions_.size(); ++i) {
    const int src = mimic_source_[i];
    if (src < 0) { continue; }
    if (std::isnan(hw_cmd_positions_[src])) { continue; }

    const double mimic_cmd =
      mimic_multiplier_[i] * hw_cmd_positions_[src] + mimic_offset_[i];
    std::ostringstream oss;
    oss << "P " << i << " " << mimic_cmd << "\n";
    send_command(oss.str());
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Serial port helpers
// ═══════════════════════════════════════════════════════════════════════════
bool ArduinoHardwareInterface::open_serial_port()
{
  serial_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) { return false; }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) { close_serial_port(); return false; }

  speed_t speed = B115200;
  switch (baud_rate_) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B115200; break;
  }
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= CLOCAL | CREAD;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;

  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    close_serial_port();
    return false;
  }
  return true;
}

void ArduinoHardwareInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoHardwareInterface::send_command(const std::string & cmd)
{
  if (serial_fd_ < 0) { return false; }
  ssize_t written = ::write(serial_fd_, cmd.c_str(), cmd.size());
  return written == static_cast<ssize_t>(cmd.size());
}

std::string ArduinoHardwareInterface::read_line()
{
  if (serial_fd_ < 0) { return {}; }
  std::string line;
  char c = 0;
  while (true) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n <= 0) { break; }
    if (c == '\n') { break; }
    line += c;
  }
  return line;
}

}  // namespace patrick_manipulation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  patrick_manipulation::ArduinoHardwareInterface,
  hardware_interface::SystemInterface)
