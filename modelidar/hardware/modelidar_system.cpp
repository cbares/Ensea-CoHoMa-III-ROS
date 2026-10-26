// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modelidar/modelidar_system.hpp"

#include <stdexcept>
#include <string>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "modelidar/modelidar_comms.hpp"

namespace modelidar
{

namespace
{
/// Consecutive failed exchanges (timeout, garbled line) tolerated before an ERROR.
constexpr int MAX_FAILURES = 10;

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & default_value = "")
{
  auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    if (default_value.empty()) {
      throw std::runtime_error("missing hardware parameter '" + name + "'");
    }
    return default_value;
  }
  return it->second;
}
}  // namespace

hardware_interface::CallbackReturn ModelidarSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto & info = params.hardware_info;

  // retrieve parameters from URDF (ros2_control section)
  try {
    cfg_.left_wheel_name = param(info, "left_wheel_joint");  // also in the controllers yaml file
    cfg_.right_wheel_name = param(info, "right_wheel_joint");  // also in the controllers yaml file
    cfg_.device = param(info, "device");
    cfg_.baud_rate = std::stoi(param(info, "baud_rate", "115200"));
    cfg_.timeout_ms = std::stoi(param(info, "timeout", "50"));
    cfg_.enc_counts_per_rev = std::stoi(param(info, "enc_counts_per_rev", "544"));

    const char * gains[] = {"kp", "ki", "kff", "dmin"};
    int given = 0;
    for (const char * g : gains) {
      given += info.hardware_parameters.count(g);
    }
    if (given == 4) {
      cfg_.send_gains = true;
      cfg_.kp = hardware_interface::stod(param(info, "kp"));
      cfg_.ki = hardware_interface::stod(param(info, "ki"));
      cfg_.kff = hardware_interface::stod(param(info, "kff"));
      cfg_.dmin = hardware_interface::stod(param(info, "dmin"));
    } else if (given != 0) {
      RCLCPP_FATAL(get_logger(), "Give all four gains (kp, ki, kff, dmin) or none.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Bad hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.enc_counts_per_rev <= 0) {
    RCLCPP_FATAL(get_logger(), "enc_counts_per_rev must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  right_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info.joints)
  {
    // Modelidar has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(get_logger(), "Initialized ModelidarSystemHardware");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// Lifecycle (ros2_control):
//   on_configure  opens the serial link: states are read from then on (read() also runs
//                 while INACTIVE)
//   on_activate   motors stopped and gains sent, then velocity commands are accepted
//   on_deactivate stops the motors, the link stays open
//   on_cleanup / on_shutdown / on_error  stop the motors (if possible) and close the link
// FAILURE leaves the component in its previous state (the transition can be retried),
// ERROR goes through on_error.

hardware_interface::CallbackReturn ModelidarSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BoardState state;
  try {
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    if (!comms_.query(state)) {
      RCLCPP_ERROR(get_logger(), "No answer from the board on %s", cfg_.device.c_str());
      comms_.disconnect();
      return hardware_interface::CallbackReturn::FAILURE;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not open %s at %d baud: %s",
      cfg_.device.c_str(), cfg_.baud_rate, e.what());
    comms_.disconnect();
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Wheel positions start at 0 for the controllers.
  comms_.take_reset();
  left_wheel_.zero(state.left.position);
  right_wheel_.zero(state.right.position);
  failures_ = 0;
  apply_state(state);
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BoardState state;
  try {
    // Start stopped: also clears a STALL fault left by a previous session.
    if (!comms_.stop(state)) {
      RCLCPP_ERROR(get_logger(), "No answer from the board to the stop command");
      return hardware_interface::CallbackReturn::FAILURE;
    }
    if (cfg_.send_gains &&
      !comms_.set_gains(cfg_.kp, cfg_.ki, cfg_.kff, cfg_.dmin, state))
    {
      RCLCPP_WARN(get_logger(), "Could not set the velocity gains, firmware defaults kept");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Serial error during activation: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  failures_ = 0;
  apply_state(state);
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Always SUCCESS: staying ACTIVE would be worse. The firmware also stops the
  // motors by itself after 500 ms without velocity command.
  stop_motors();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motors();
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully shut down!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    get_logger(), "Error in state '%s': motors stopped, serial link closed",
    previous_state.label().c_str());
  stop_motors();
  comms_.disconnect();

  // Back to UNCONFIGURED: configure + activate again to recover.
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ModelidarSystemHardware::stop_motors()
{
  if (!comms_.connected()) {
    return;
  }
  try {
    BoardState state;
    if (!comms_.stop(state)) {
      RCLCPP_WARN(get_logger(), "No answer to the stop command (the board stops after 500 ms)");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Could not stop the motors: %s", e.what());
  }
}

void ModelidarSystemHardware::apply_state(const BoardState & state)
{
  if (comms_.take_reset()) {
    RCLCPP_WARN(get_logger(), "The board restarted: wheel positions kept continuous");
    left_wheel_.firmware_restarted();
    right_wheel_.firmware_restarted();
  }

  set_state(left_wheel_.position_name, left_wheel_.position(state.left.position));
  set_state(right_wheel_.position_name, right_wheel_.position(state.right.position));
  set_state(left_wheel_.velocity_name, left_wheel_.velocity(state.left.velocity));
  set_state(right_wheel_.velocity_name, right_wheel_.velocity(state.right.velocity));

  if (state.owner != owner_) {
    if (state.owner == "USB") {
      RCLCPP_INFO(get_logger(), "ROS drives the motors again");
    } else if (state.owner == "RC") {
      RCLCPP_WARN(get_logger(), "Radio control took over: velocity commands ignored");
    } else {
      RCLCPP_WARN(
        get_logger(), "Radio link lost during override (%s): motors stopped, commands ignored",
        state.owner.c_str());
    }
    owner_ = state.owner;
  }
  if (state.fault != fault_) {
    if (state.fault == "-") {
      RCLCPP_INFO(get_logger(), "Board fault cleared");
    } else if (state.fault == "STALL") {
      RCLCPP_ERROR(
        get_logger(), "Wheel stalled: motors stopped until a zero velocity command");
    } else {
      RCLCPP_WARN(get_logger(), "Board fault: %s", state.fault.c_str());
    }
    fault_ = state.fault;
  }
}

hardware_interface::return_type ModelidarSystemHardware::exchange_failed(const char * what)
{
  ++failures_;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000, "No valid answer to %s (%d in a row)", what, failures_);
  if (failures_ >= MAX_FAILURES) {
    RCLCPP_ERROR(get_logger(), "Board not answering: %d failed exchanges", failures_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModelidarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  BoardState state;
  try {
    if (!comms_.query(state)) {
      return exchange_failed("state query");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Serial error while reading: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  failures_ = 0;
  apply_state(state);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 500,
    "Read: L pos %ld vel %.1f, R pos %ld vel %.1f (ticks, ticks/s) C:%s F:%s",
    state.left.position, state.left.velocity, state.right.position, state.right.velocity,
    state.owner.c_str(), state.fault.c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModelidarSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double left = get_command(left_wheel_.velocity_name);
  const double right = get_command(right_wheel_.velocity_name);

  BoardState state;
  try {
    // Regulated by the board (rad/s). Sent every cycle: it also keeps the
    // firmware USB timeout (500 ms) from stopping the motors.
    if (!comms_.set_velocity(left, right, state)) {
      return exchange_failed("velocity command");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Serial error while writing: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  failures_ = 0;
  apply_state(state);

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Write: L %.3f rad/s, R %.3f rad/s, duty applied %d,%d",
    left, right, state.duty_left, state.duty_right);

  return hardware_interface::return_type::OK;
}

}  // namespace modelidar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  modelidar::ModelidarSystemHardware, hardware_interface::SystemInterface)
