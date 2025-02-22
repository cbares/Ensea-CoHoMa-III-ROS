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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "modelidar/modelidar_comms.hpp"

namespace modelidar
{

hardware_interface::CallbackReturn ModelidarSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // retrieve parameters from URDF (ros2_control section)
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_joint"]; //this value is also in controllers yaml file
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_joint"]; //this value is also in controllers yaml file
  cfg_.loop_rate = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
  left_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  right_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
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

hardware_interface::CallbackReturn ModelidarSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
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
  try {
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout);
  } catch (const std::runtime_error &) {
    RCLCPP_FATAL(get_logger(), "Could not connect to device %s at %d baud", cfg_.device.c_str(), cfg_.baud_rate);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ModelidarSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ModelidarSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double posL, posR, velL, velR;
  std::string resp = comms_.get_state_values(posL, posR, velL, velR);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";
  ss << std::fixed << std::setprecision(2);

  try{
    set_state(left_wheel_.position_name, posL);
    set_state(right_wheel_.position_name, posR);
    set_state(left_wheel_.velocity_name, velL);
    set_state(right_wheel_.velocity_name, velR);
  } catch (std::out_of_range &) {
    ss << std::endl << "State interface not found during read :" << left_wheel_.position_name << " or " << right_wheel_.velocity_name;
  }
  
  ss << std::endl << "Left wheel - Position: " << posL << " Velocity: " << velL;
  ss << std::endl << "Right wheel - Position: " << posR << " Velocity: " << velR;

  ss << std::endl << "Response: " << resp;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ModelidarSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // call to set_motor_values(left, right);

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  double left = 0.0;
  double right = 0.0;

  ss << "Writing commands:";
  try{
    left = get_command(left_wheel_.velocity_name);
    right = get_command(right_wheel_.velocity_name);
  } catch (std::out_of_range &) {
       ss << std::endl << "Command interface not found during write"; 
  }

  std::string response = comms_.set_motor_speed(left, right);
  ss << std::endl << "Response: " << response;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace modelidar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  modelidar::ModelidarSystemHardware, hardware_interface::SystemInterface)
