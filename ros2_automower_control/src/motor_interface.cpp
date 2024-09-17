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

#include "ros2_automower_control/motor_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_automower_control
{
hardware_interface::CallbackReturn MotorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
  cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
  cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
  cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  cfg_.enc_counts_per_rev =
    std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn MotorInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Activating ...please wait...");

  comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  std::pair<bool, std::string> response = comms_.setPidValues(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  if (response.first == true) {
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MotorInterface"), response.second);
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MotorInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Configuring ...please wait...");

  comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  std::pair<bool, std::string> response = comms_.setPidValues(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  if (response.first == true) {
    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MotorInterface"), response.second);
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn MotorInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Deactivating ...please wait...");

  comms_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotorInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  std::pair<bool, std::string>response = comms_.readEncoderValues(wheel_l_.enc, wheel_r_.enc);
  if (response.first == true){
  double delta_seconds = period.seconds();

  float pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calcEncAngle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calcEncAngle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MotorInterface"), response.second);
    return hardware_interface::return_type::ERROR;
  };
}

hardware_interface::return_type ros2_automower_control::MotorInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Writing...");

  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
  std::pair<bool, std::string>response = comms_.setMotorValues(motor_l_counts_per_loop, motor_r_counts_per_loop);
  if (response.first == true){
  RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MotorInterface"), response.second);
    return hardware_interface::return_type::ERROR;
  };
}

}  // namespace ros2_automower_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_automower_control::MotorInterface, hardware_interface::SystemInterface)