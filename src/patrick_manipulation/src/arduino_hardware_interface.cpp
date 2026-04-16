// Copyright 2026 Patrick Robot Project
// SPDX-License-Identifier: MIT

#include "patrick_manipulation/arduino_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
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

  // Read hardware parameters from URDF <ros2_control> block
  port_name_ = info_.hardware_parameters.count("port_name")
    ? info_.hardware_parameters.at("port_name") : "/dev/ttyACM0";
  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;

  const auto num_joints = info_.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_cmd_efforts_.resize(num_joints, 0.0);

  // Validate joint interfaces
  for (const auto & joint : info_.joints) {
    // Expect exactly one command interface (effort)
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ArduinoHardwareInterface"),
        "Joint '%s' must have exactly 1 command interface, found %zu",
        joint.name.c_str(), joint.command_interfaces.size());
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
//  Lifecycle: on_activate  — enable motors
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send enable command to Arduino
  send_command("E\n");

  // Read initial positions
  send_command("R\n");
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    auto line = read_line();
    if (!line.empty()) {
      // Parse "S <joint_idx> <pos> <vel>"
      int idx = 0;
      double pos = 0.0, vel = 0.0;
      std::istringstream iss(line);
      char tag;
      if (iss >> tag >> idx >> pos >> vel) {
        if (idx >= 0 && idx < static_cast<int>(hw_positions_.size())) {
          hw_positions_[idx] = pos;
          hw_velocities_[idx] = vel;
        }
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"), "Motors enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_deactivate  — disable motors
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_command("D\n");  // Disable motors
  RCLCPP_INFO(
    rclcpp::get_logger("ArduinoHardwareInterface"), "Motors disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_cleanup  — close serial port
// ═══════════════════════════════════════════════════════════════════════════
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
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_cmd_efforts_[i]);
  }
  return command_interfaces;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Real-time loop: read
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Request state from Arduino
  send_command("R\n");

  // Parse all joint state lines
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    auto line = read_line();
    if (line.empty()) {
      continue;
    }
    // Expected format: "S <joint_idx> <position> <velocity>"
    int idx = 0;
    double pos = 0.0, vel = 0.0;
    std::istringstream iss(line);
    char tag;
    if (iss >> tag >> idx >> pos >> vel) {
      if (idx >= 0 && idx < static_cast<int>(hw_positions_.size())) {
        hw_positions_[idx] = pos;
        hw_velocities_[idx] = vel;
      }
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
  // Send effort commands for each joint
  for (size_t i = 0; i < hw_cmd_efforts_.size(); ++i) {
    std::ostringstream oss;
    oss << "W " << i << " " << hw_cmd_efforts_[i] << "\n";
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
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(serial_fd_, &tty) != 0) {
    close_serial_port();
    return false;
  }

  // Map baud rate
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

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit characters
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag |= CLOCAL | CREAD;  // Enable receiver, ignore modem controls

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
  tty.c_lflag = 0;   // Raw mode
  tty.c_oflag = 0;   // Raw output

  tty.c_cc[VMIN]  = 0;   // Non-blocking read
  tty.c_cc[VTIME] = 1;   // 100ms timeout

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
  if (serial_fd_ < 0) {
    return false;
  }
  ssize_t written = ::write(serial_fd_, cmd.c_str(), cmd.size());
  return written == static_cast<ssize_t>(cmd.size());
}

std::string ArduinoHardwareInterface::read_line()
{
  if (serial_fd_ < 0) {
    return {};
  }

  std::string line;
  char c = 0;
  while (true) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n <= 0) {
      break;  // Timeout or error
    }
    if (c == '\n') {
      break;
    }
    line += c;
  }
  return line;
}

}  // namespace patrick_manipulation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  patrick_manipulation::ArduinoHardwareInterface,
  hardware_interface::SystemInterface)
