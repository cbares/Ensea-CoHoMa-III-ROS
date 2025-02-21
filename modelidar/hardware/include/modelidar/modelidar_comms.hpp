#ifndef MODELIBOT_COMMS_HPP_
#define MODELIBOT_COMMS_HPP_

/**
 * @file modelibot_comms.hpp
 * @brief Header file for communication module for mdoelibot
 */

#define HAVE_STDINT_H 1

#include <sstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <regex>
#include <string>
#include <iostream>
#include <cmath>

#include <serial.h>


// Comm examples:
// Send a command: #0600,0700
// Ask for state: ?
// State return: !0500,0500 L:+0,+0 R:+0,+0

constexpr auto& CMD_PATTERN = "#%04d,%04d\r";
constexpr auto& QUERY = "?\r";
constexpr auto& STATE_PATTERN = "!%04d,%04d L:%+d,%+d R:%+d,%+d\r";
constexpr auto& STATE_REGEX = "!(\\d{4}),(\\d{4}) L:([+-]?\\d+),([+-]?\\d+) R:([+-]?\\d+),([+-]?\\d+)";
constexpr auto& ENCODER_REGEX = "!\\d{4},\\d{4} L:[+-]?\\d+,([+-]?\\d+) R:[+-]?\\d+,([+-]?\\d+)";

const double STEPS = 1000; // PWM steps
// Motor specs
const double Kv = 295 / 6 * 2 * M_PI / 60; // DAGU WT341 Motor: 295 RPM@6V
const double Vo = 7.0; // Operating voltage (min)
const double Wmax = Kv * Vo; // Max angular velocity
const int counts_per_rev = 544; // Encoder counts per revolution
const double VEL_PERIOD = 0.1; // Velocity control frequency, in seconds


/**
 * @class ModelibotComms
 * @brief Class for STM32 communication
 */
class ModelibotComms
{
private:
    serial::Serial serial_conn_;
    serial::Timeout timeout_ms_;
    bool print_output_;

public:

  /**
   * @brief Default constructor
   */
  ModelibotComms(): serial_conn_(), timeout_ms_(){
    print_output_ = false;
  }

  /**
   * @brief Constructor
   * @param serial_device The serial device to connect to
   * @param baud_rate The baud rate to use for the connection
   * @param timeout_ms The timeout value in milliseconds
   */
  ModelibotComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms): serial_conn_(), timeout_ms_(){
    print_output_ = false;
    connect(serial_device, baud_rate, timeout_ms);
  }

  /**
   * @brief Connects to the serial device
   * @param serial_device The serial device to connect to
   * @param baud_rate The baud rate to use for the connection
   * @param timeout_ms The timeout value in milliseconds
   */
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    auto t = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(t);

    try{
        serial_conn_.open();
    }
    catch (const serial::IOException&){
        std::cout << "Error! Could not open device \"" << serial_device << "\"!)"  << std::endl;
        throw std::runtime_error("Could not open device");
    }
  }

  /**
   * @brief Disconnects from the serial device
   */
  void disconnect()
  {
    serial_conn_.flush();
    serial_conn_.close();
  }

  /**
   * @brief Checks if connected to the serial device
   * @return True if connected, false otherwise
   */
  bool connected() const
  {
    return serial_conn_.isOpen();
  }

  void set_print_output()
  {
    print_output_ = true;
  }

  void reset_print_output()
  {
    print_output_ = false;
  }

  /**
   * @brief Sends a message to the serial device and receives the response
   * @param msg_to_send The message to send
   * @return The response from the serial device
   */
  std::string send_msg(const std::string &msg_to_send)
  {
  
    serial_conn_.flush(); // Just in case
    serial_conn_.write(msg_to_send);

    // TODO: throw exception instead of printing error message
    std::string response;
    // Responses end with \r\n so we will read up to (and including) the \n.
    response = serial_conn_.readline();
    

    if (print_output_)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void read_state(int &val_L, int &val_R)
  {
    std::string response = send_msg(QUERY);

    // State return:      !0500,0500 L:+0,+0 R:+0,+0
    // Encoder positions:                 ^^      ^^

    std::regex encoder_regex(ENCODER_REGEX);
    std::smatch matches;
    if(std::regex_search(response, matches, encoder_regex)){
      // matches[0] is the whole string, matches[1] is the first group, etc.
      val_L = std::stoi(matches[1]);
      val_R = std::stoi(matches[2]);
      return;
    }
  }

  void read_full_state(int &pos_L, int &pos_R, int &vel_L, int &vel_R)
  {
    std::string response = send_msg(QUERY);

    // State return:      !0500,0500 L:+0,+0 R:+0,+0
    // Groups:              -1-  -2-   -3-4-   -5-6-
    // Command positions:  ^^^^ ^^^^
    // Speed positions:                ^^      ^^
    // Encoder positions:                 ^^      ^^

    std::regex state_regex(STATE_REGEX);
    std::smatch matches;
    if(std::regex_search(response, matches, state_regex)){
      // matches[0] is the whole string, matches[1] is the first group, etc.
      pos_L = std::stoi(matches[4]);
      pos_R = std::stoi(matches[6]);
      vel_L = std::stoi(matches[3]);
      vel_R = std::stoi(matches[5]);
      return;
    }
  }

  std::string get_state_values(double &pos_L, double &pos_R, double &vel_L, double &vel_R)
  {
    int pos_L_enc, pos_R_enc, vel_L_enc, vel_R_enc;
    read_full_state(pos_L_enc, pos_R_enc, vel_L_enc, vel_R_enc);

    pos_L = ((double)pos_L_enc) / counts_per_rev * 2 * M_PI;
    pos_R = ((double)pos_R_enc) / counts_per_rev * 2 * M_PI;
    vel_L = ((double)vel_L_enc) / counts_per_rev * 2 * M_PI / VEL_PERIOD;
    vel_R = ((double)vel_R_enc) / counts_per_rev * 2 * M_PI / VEL_PERIOD;

    std::stringstream ss;
    ss << "L: " << pos_L_enc << " " << vel_L_enc << " R: " << pos_R_enc << " " << vel_R_enc;
    return ss.str();
  }

  std::string set_motor_values(int val_L, int val_R)
  {
    std::stringstream ss;
    ss << "#" 
      << std::setfill('0') << std::setw(4)
      << val_L << "," 
      << std::setfill('0') << std::setw(4)
      << val_R << "\r";
    send_msg(ss.str());
    return (ss.str());
  }

  std::string set_motor_speed(double speed_L, double speed_R)
  {
    // speed in rad/s, need to be convert to PWM duty cycle beteween 0 and STEPS
    // speed / Wmax => [-1, +1]
    // speed / Wmax * STEPS/2 + STEPS/2 => [0, STEPS]
    int pwm_L = (speed_L / Wmax) * STEPS/2 + STEPS/2;
    int pwm_R = (speed_R / Wmax) * STEPS/2 + STEPS/2;
    if (pwm_L > STEPS) pwm_L = STEPS;
    if (pwm_L < 0) pwm_L = 0;
    if (pwm_R > STEPS) pwm_R = STEPS;
    if (pwm_R < 0) pwm_R = 0;

    return set_motor_values(pwm_L, pwm_R);;
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

};

#endif  // MODELIBOT_COMMS_HPP_
