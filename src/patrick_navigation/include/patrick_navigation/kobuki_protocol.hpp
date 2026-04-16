// =============================================================================
// kobuki_protocol.hpp — Minimal Kobuki serial protocol (USB @ 115200 baud)
//
// Packet format (both TX and RX):
//   [0xAA][0x55][Length][Payload ...][Checksum]
//   Checksum = XOR of all bytes from Length through end of Payload
//
// Reference: Kobuki protocol specification v1.1
// =============================================================================
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

namespace kobuki {

// ─── Constants ──────────────────────────────────────────────────────────────
static constexpr uint8_t HEADER_0 = 0xAA;
static constexpr uint8_t HEADER_1 = 0x55;

// Sub-payload IDs (feedback)
static constexpr uint8_t SUBID_BASIC_SENSOR = 0x01;
static constexpr uint8_t SUBID_DOCK_IR      = 0x03;
static constexpr uint8_t SUBID_INERTIAL     = 0x04;
static constexpr uint8_t SUBID_CLIFF        = 0x05;
static constexpr uint8_t SUBID_CURRENT      = 0x06;
static constexpr uint8_t SUBID_HW_VERSION   = 0x0A;
static constexpr uint8_t SUBID_FW_VERSION   = 0x0B;
static constexpr uint8_t SUBID_GYRO_RAW     = 0x0D;
static constexpr uint8_t SUBID_GPIO         = 0x10;
static constexpr uint8_t SUBID_UUID         = 0x13;

// Sub-payload IDs (commands)
static constexpr uint8_t CMD_BASE_CONTROL   = 0x01;
static constexpr uint8_t CMD_SOUND          = 0x03;
static constexpr uint8_t CMD_SOUND_SEQ      = 0x04;
static constexpr uint8_t CMD_REQUEST_EXTRA  = 0x09;
static constexpr uint8_t CMD_GPIO           = 0x0C;
static constexpr uint8_t CMD_SET_CONTROLLER = 0x0D;
static constexpr uint8_t CMD_GET_CONTROLLER = 0x0E;

// Bumper bit masks
static constexpr uint8_t BUMPER_RIGHT  = 0x01;
static constexpr uint8_t BUMPER_CENTER = 0x02;
static constexpr uint8_t BUMPER_LEFT   = 0x04;

// Cliff bit masks
static constexpr uint8_t CLIFF_RIGHT  = 0x01;
static constexpr uint8_t CLIFF_CENTER = 0x02;
static constexpr uint8_t CLIFF_LEFT   = 0x04;

// Wheel drop bit masks
static constexpr uint8_t WHEEL_DROP_RIGHT = 0x01;
static constexpr uint8_t WHEEL_DROP_LEFT  = 0x02;

// Encoder ticks per revolution
static constexpr double TICKS_PER_REV = 11724.41658;
// Tick to radian conversion
static constexpr double TICK_TO_RAD = 2.0 * M_PI / TICKS_PER_REV;

// ─── Parsed sensor data ────────────────────────────────────────────────────
struct BasicSensorData {
  uint16_t timestamp = 0;
  uint8_t  bumper = 0;
  uint8_t  wheel_drop = 0;
  uint8_t  cliff = 0;
  uint16_t left_encoder = 0;
  uint16_t right_encoder = 0;
  int8_t   left_pwm = 0;
  int8_t   right_pwm = 0;
  uint8_t  buttons = 0;
  uint8_t  charger = 0;
  uint8_t  battery = 0;        // raw voltage: battery * 0.1 = volts
  uint8_t  overcurrent = 0;
};

struct InertialData {
  int16_t angle = 0;           // in hundredths of degree
  int16_t angle_rate = 0;      // in hundredths of degree/s
};

struct CliffData {
  uint16_t bottom[3] = {0, 0, 0};  // right, center, left (raw ADC)
};

struct KobukiFeedback {
  bool has_basic = false;
  bool has_inertial = false;
  bool has_cliff = false;
  BasicSensorData basic;
  InertialData    inertial;
  CliffData       cliff;
};

// ─── Encoder delta helper (handles 16-bit overflow) ─────────────────────────
inline int16_t encoder_delta(uint16_t new_val, uint16_t old_val) {
  int32_t diff = static_cast<int32_t>(new_val) - static_cast<int32_t>(old_val);
  if (diff > 32768)  diff -= 65536;
  if (diff < -32768) diff += 65536;
  return static_cast<int16_t>(diff);
}

// ─── Serial port wrapper ───────────────────────────────────────────────────
class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { close_port(); }

  bool open_port(const std::string & device, int baud = B115200) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) { close_port(); return false; }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;

    // Non-blocking with min 0 bytes
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close_port(); return false; }

    return true;
  }

  void close_port() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const { return fd_ >= 0; }

  // Read available bytes into buffer, returns count
  int read_bytes(uint8_t * buf, size_t max_len) {
    if (fd_ < 0) return -1;
    return static_cast<int>(::read(fd_, buf, max_len));
  }

  // Write bytes
  int write_bytes(const uint8_t * buf, size_t len) {
    if (fd_ < 0) return -1;
    return static_cast<int>(::write(fd_, buf, len));
  }

private:
  int fd_ = -1;
};

// ─── Packet parser (accumulates bytes, extracts complete packets) ───────────
class PacketParser {
public:
  // Feed raw bytes from serial. Returns true if at least one complete
  // feedback packet was parsed (results stored in last_feedback_).
  bool feed(const uint8_t * data, size_t len) {
    bool got_packet = false;
    for (size_t i = 0; i < len; ++i) {
      uint8_t b = data[i];
      switch (state_) {
        case State::HEADER0:
          if (b == HEADER_0) state_ = State::HEADER1;
          break;
        case State::HEADER1:
          if (b == HEADER_1) { state_ = State::LENGTH; }
          else { state_ = State::HEADER0; }
          break;
        case State::LENGTH:
          payload_len_ = b;
          checksum_ = b;
          payload_.clear();
          payload_.reserve(payload_len_);
          if (payload_len_ == 0) { state_ = State::HEADER0; }
          else { state_ = State::PAYLOAD; }
          break;
        case State::PAYLOAD:
          payload_.push_back(b);
          checksum_ ^= b;
          if (payload_.size() >= payload_len_) {
            state_ = State::CHECKSUM;
          }
          break;
        case State::CHECKSUM:
          if (checksum_ == b) {
            parse_payload(payload_);
            got_packet = true;
          }
          state_ = State::HEADER0;
          break;
      }
    }
    return got_packet;
  }

  const KobukiFeedback & feedback() const { return feedback_; }

private:
  enum class State { HEADER0, HEADER1, LENGTH, PAYLOAD, CHECKSUM };
  State state_ = State::HEADER0;
  uint8_t payload_len_ = 0;
  uint8_t checksum_ = 0;
  std::vector<uint8_t> payload_;
  KobukiFeedback feedback_;

  void parse_payload(const std::vector<uint8_t> & p) {
    feedback_.has_basic = false;
    feedback_.has_inertial = false;
    feedback_.has_cliff = false;

    size_t idx = 0;
    while (idx + 1 < p.size()) {
      uint8_t sub_id  = p[idx];
      uint8_t sub_len = p[idx + 1];
      idx += 2;
      if (idx + sub_len > p.size()) break;

      switch (sub_id) {
        case SUBID_BASIC_SENSOR:
          if (sub_len >= 15) {
            auto & s = feedback_.basic;
            s.timestamp     = read_u16(p, idx);
            s.bumper        = p[idx + 2];
            s.wheel_drop    = p[idx + 3];
            s.cliff         = p[idx + 4];
            s.left_encoder  = read_u16(p, idx + 5);
            s.right_encoder = read_u16(p, idx + 7);
            s.left_pwm      = static_cast<int8_t>(p[idx + 9]);
            s.right_pwm     = static_cast<int8_t>(p[idx + 10]);
            s.buttons       = p[idx + 11];
            s.charger       = p[idx + 12];
            s.battery       = p[idx + 13];
            s.overcurrent   = p[idx + 14];
            feedback_.has_basic = true;
          }
          break;

        case SUBID_INERTIAL:
          if (sub_len >= 4) {
            feedback_.inertial.angle      = read_i16(p, idx);
            feedback_.inertial.angle_rate  = read_i16(p, idx + 2);
            feedback_.has_inertial = true;
          }
          break;

        case SUBID_CLIFF:
          if (sub_len >= 6) {
            feedback_.cliff.bottom[0] = read_u16(p, idx);
            feedback_.cliff.bottom[1] = read_u16(p, idx + 2);
            feedback_.cliff.bottom[2] = read_u16(p, idx + 4);
            feedback_.has_cliff = true;
          }
          break;

        default:
          break;  // skip unknown sub-payloads
      }
      idx += sub_len;
    }
  }

  static uint16_t read_u16(const std::vector<uint8_t> & p, size_t i) {
    return static_cast<uint16_t>(p[i]) | (static_cast<uint16_t>(p[i + 1]) << 8);
  }
  static int16_t read_i16(const std::vector<uint8_t> & p, size_t i) {
    return static_cast<int16_t>(read_u16(p, i));
  }
};

// ─── Command builder ───────────────────────────────────────────────────────
class CommandBuilder {
public:
  // Build a base control command packet from linear.x (m/s) and angular.z (rad/s)
  // Returns the full packet bytes ready to write to serial.
  //
  // Kobuki uses (speed, radius) internally:
  //   speed  = 1000 * linear.x   (mm/s, int16)
  //   radius = speed / angular.z (mm, int16) if angular.z != 0
  //   Special: radius = 0 means straight, radius = 1 means in-place rotation
  static std::vector<uint8_t> base_control(double linear_x, double angular_z) {
    int16_t speed  = static_cast<int16_t>(std::round(linear_x * 1000.0));
    int16_t radius = 0;

    const double eps = 1e-6;
    if (std::abs(linear_x) < eps && std::abs(angular_z) > eps) {
      // Pure rotation: speed = wheel_separation/2 * angular_z * 1000
      speed  = static_cast<int16_t>(std::round((230.0 / 2.0) * angular_z));
      radius = 1;  // special value = in-place rotation
    } else if (std::abs(angular_z) > eps) {
      // Arc motion
      double r = linear_x / angular_z * 1000.0;  // mm
      r = std::max(-32768.0, std::min(32767.0, r));
      radius = static_cast<int16_t>(std::round(r));
    }
    // else: straight line (radius = 0)

    // Build sub-payload: [CMD_BASE_CONTROL][4][speed_lo][speed_hi][radius_lo][radius_hi]
    uint8_t payload[6];
    payload[0] = CMD_BASE_CONTROL;
    payload[1] = 0x04;
    payload[2] = static_cast<uint8_t>(speed & 0xFF);
    payload[3] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    payload[4] = static_cast<uint8_t>(radius & 0xFF);
    payload[5] = static_cast<uint8_t>((radius >> 8) & 0xFF);

    return build_packet(payload, 6);
  }

  // Motor power: state 0=off, 1=on  (not in minimal spec but useful)
  // This uses the sound-sequence command trick to wake up / put to sleep
  // Actually Kobuki doesn't have a dedicated motor power command in protocol.
  // Motor power is controlled via a sustained base_control stream.
  // If no command received for ~250ms, motors coast.

private:
  static std::vector<uint8_t> build_packet(const uint8_t * payload, size_t len) {
    std::vector<uint8_t> pkt;
    pkt.reserve(4 + len);
    pkt.push_back(HEADER_0);
    pkt.push_back(HEADER_1);
    pkt.push_back(static_cast<uint8_t>(len));

    uint8_t cs = static_cast<uint8_t>(len);
    for (size_t i = 0; i < len; ++i) {
      pkt.push_back(payload[i]);
      cs ^= payload[i];
    }
    pkt.push_back(cs);
    return pkt;
  }
};

}  // namespace kobuki
