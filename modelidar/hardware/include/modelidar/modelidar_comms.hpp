#ifndef MODELIDAR__MODELIDAR_COMMS_HPP_
#define MODELIDAR__MODELIDAR_COMMS_HPP_

/**
 * @file modelidar_comms.hpp
 * @brief Serial link to the modelidar firmware (firmware/README.md, "Protocole série").
 *
 * Commands (each one answered by exactly one line):
 *   ?               query the state
 *   V<l>,<r>        regulated wheel velocity, in mrad/s
 *   /               stop, wheels free
 *   *               brake
 *   K<kp>,<ki>,<kff>,<dmin>   set the velocity loop gains: answered by "K:kp,ki,kff,dmin"
 *                   instead of the state, and not answered at all if the gains are refused
 *
 * State line:  !0700,0600 L:+600,+420 R:+42.857,+210 C:USB F:-
 *   - duty cycle applied (0..1000, 500 = stopped)
 *   - per wheel: velocity (encoder ticks/s, may have decimals), position (ticks)
 *   - C: who drives (USB, RC, LOST)
 *   - F: active fault (-, USB_TIMEOUT, STALL)
 *
 * At boot the firmware also sends, unprompted, "MCU started (reset: ...)": the positions
 * restart at 0.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#ifndef HAVE_STDINT_H
#define HAVE_STDINT_H 1  // serial.h: use <stdint.h>
#endif
#include <serial.h>

namespace modelidar
{

/// Wheel velocity command limit accepted by the firmware (rad/s).
constexpr double MAX_WHEEL_SPEED = 50.0;

/// Start of the line sent by the firmware at boot.
constexpr const char * BOOT_MESSAGE = "MCU started";

struct WheelReading
{
  double velocity = 0.0;  ///< encoder ticks per second
  long position = 0;      ///< encoder ticks since the firmware started
};

struct BoardState
{
  int duty_left = 500;
  int duty_right = 500;
  WheelReading left;
  WheelReading right;
  std::string owner;  ///< USB, RC or LOST
  std::string fault;  ///< -, USB_TIMEOUT or STALL
};

/// Velocity loop gains (firmware/README.md, "Régulation de vitesse").
struct Gains
{
  double kp = 0.0;
  double ki = 0.0;
  double kff = 0.0;
  double dmin = 0.0;
};

/// Parses a state line ("!AAAA,BBBB L:v,p R:v,p C:x F:y"). Returns false if it is not one.
inline bool parse_state(const std::string & line, BoardState & out)
{
  BoardState s;
  char owner[16] = {0};
  char fault[32] = {0};
  int end = -1;
  const int n = std::sscanf(
    line.c_str(), "!%d,%d L:%lf,%ld R:%lf,%ld C:%15s F:%31s %n",
    &s.duty_left, &s.duty_right,
    &s.left.velocity, &s.left.position,
    &s.right.velocity, &s.right.position,
    owner, fault, &end);
  if (n != 8 || end < 0 || static_cast<size_t>(end) != line.size()) {
    return false;
  }
  s.owner = owner;
  s.fault = fault;
  out = s;
  return true;
}

/// Parses a gains line ("K:kp,ki,kff,dmin"). Returns false if it is not one.
inline bool parse_gains(const std::string & line, Gains & out)
{
  Gains g;
  int end = -1;
  const int n = std::sscanf(
    line.c_str(), "K:%lf,%lf,%lf,%lf %n", &g.kp, &g.ki, &g.kff, &g.dmin, &end);
  if (n != 4 || end < 0 || static_cast<size_t>(end) != line.size()) {
    return false;
  }
  out = g;
  return true;
}

/// True if the gains are equal up to the firmware precision (f32).
inline bool same_gains(const Gains & a, const Gains & b)
{
  auto close = [](double x, double y) {
      return std::abs(x - y) <= 1e-6 + 1e-4 * std::abs(y);
    };
  return close(a.kp, b.kp) && close(a.ki, b.ki) && close(a.kff, b.kff) &&
         close(a.dmin, b.dmin);
}

/// Velocity command frame, in mrad/s, clamped to the firmware limit.
inline std::string velocity_frame(double left_rad_s, double right_rad_s)
{
  auto to_mrad = [](double v) {
      if (!std::isfinite(v)) {
        v = 0.0;
      }
      return std::lround(std::clamp(v, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED) * 1000.0);
    };
  std::ostringstream ss;
  ss << "V" << to_mrad(left_rad_s) << "," << to_mrad(right_rad_s) << "\r";
  return ss.str();
}

/**
 * @class ModelidarComms
 * @brief Serial link to the STM32 board. Serial errors are thrown as exceptions.
 */
class ModelidarComms
{
public:
  ModelidarComms() = default;

  void connect(const std::string & device, uint32_t baud_rate, uint32_t timeout_ms)
  {
    disconnect();  // opening an open port throws
    serial_conn_.setPort(device);
    serial_conn_.setBaudrate(baud_rate);
    auto t = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(t);
    serial_conn_.open();
    serial_conn_.flushInput();
    rx_tail_.clear();
  }

  /// Closes the link. Never throws: used on error and shutdown paths.
  void disconnect() noexcept
  {
    try {
      if (serial_conn_.isOpen()) {
        serial_conn_.close();
      }
    } catch (...) {
      // device already gone (unplugged): nothing left to close
    }
  }

  bool connected() const {return serial_conn_.isOpen();}

  void set_print_output(bool on) {print_output_ = on;}

  bool query(BoardState & state) {return exchange("?\r", state);}

  /// Regulated wheel velocities, in rad/s.
  bool set_velocity(double left_rad_s, double right_rad_s, BoardState & state)
  {
    return exchange(velocity_frame(left_rad_s, right_rad_s), state);
  }

  /// Stops the motors, wheels free. Also clears a STALL fault.
  bool stop(BoardState & state) {return exchange("/\r", state);}

  bool brake(BoardState & state) {return exchange("*\r", state);}

  /**
   * Sets the velocity loop gains (lost at reboot). `applied` receives the gains the board
   * now uses. False on timeout: the firmware does not answer gains it refuses.
   */
  bool set_gains(const Gains & gains, Gains & applied)
  {
    std::ostringstream ss;
    ss << "K" << gains.kp << "," << gains.ki << "," << gains.kff << "," << gains.dmin << "\r";
    std::string line;
    return request(ss.str(), 'K', line) && parse_gains(line, applied);
  }

  /// True once after the firmware restarted (its positions went back to 0).
  bool take_reset()
  {
    const bool r = reset_seen_;
    reset_seen_ = false;
    return r;
  }

  /// Last boot line received, e.g. "MCU started (reset: watchdog)".
  const std::string & boot_message() const {return boot_message_;}

private:
  static std::string strip_eol(std::string line)
  {
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    return line;
  }

  /// Records a boot line. True if `line` is one.
  bool note_boot(const std::string & line)
  {
    if (line.rfind(BOOT_MESSAGE, 0) != 0) {
      return false;
    }
    reset_seen_ = true;
    boot_message_ = line;
    return true;
  }

  /**
   * Reads what arrived since the last exchange: stale answers (dropped, so that answers
   * stay in step) and the boot line (sent unprompted, recorded). An unfinished boot line
   * is kept in rx_tail_: its end is read with the next answer.
   */
  void drain_pending()
  {
    std::string data;
    data.swap(rx_tail_);
    const size_t n = serial_conn_.available();
    if (n > 0) {
      data += serial_conn_.read(n);
    }
    size_t start = 0;
    for (size_t eol; (eol = data.find('\n', start)) != std::string::npos; start = eol + 1) {
      note_boot(strip_eol(data.substr(start, eol - start)));
    }
    const std::string rest = data.substr(start);
    const std::string boot(BOOT_MESSAGE);
    const size_t len = std::min(rest.size(), boot.size());
    if (!rest.empty() && rest.compare(0, len, boot, 0, len) == 0) {
      rx_tail_ = rest;
    }
  }

  /**
   * Sends a command and waits for its answer, the first line starting with `prefix`,
   * skipping the other lines (boot message). False on timeout.
   */
  bool request(const std::string & msg, char prefix, std::string & answer)
  {
    drain_pending();
    serial_conn_.write(msg);

    for (int i = 0; i < 4; ++i) {
      std::string line = serial_conn_.readline(256, "\n");
      if (print_output_) {
        std::cout << "Sent: " << msg << " Recv: " << line << std::endl;
      }
      if (line.empty()) {
        return false;  // timeout
      }
      if (!rx_tail_.empty()) {
        line = rx_tail_ + line;  // end of a boot line started before the command
        rx_tail_.clear();
      }
      line = strip_eol(line);
      if (note_boot(line)) {
        continue;
      }
      if (!line.empty() && line[0] == prefix) {
        answer = line;
        return true;
      }
    }
    return false;
  }

  /// Sends a command and parses its state line. False on timeout or garbled answer.
  bool exchange(const std::string & msg, BoardState & state)
  {
    std::string line;
    return request(msg, '!', line) && parse_state(line, state);
  }

  serial::Serial serial_conn_;
  bool print_output_ = false;
  bool reset_seen_ = false;
  std::string boot_message_;
  std::string rx_tail_;  ///< start of a boot line not finished yet
};

}  // namespace modelidar

#endif  // MODELIDAR__MODELIDAR_COMMS_HPP_
