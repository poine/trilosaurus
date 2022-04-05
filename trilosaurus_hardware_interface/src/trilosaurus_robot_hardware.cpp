// Copyright (c) 2022, poine
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>

#include "trilosaurus_hardware_interface/trilosaurus_robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "trilosaurus_hardware_interface/trilobot_driver.h"


namespace trilosaurus_hardware_interface
{

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <wiringPi.h>
  
//   class MotorController {

// // # Motor driver pins, via DRV8833PWP Dual H-Bridge
// // MOTOR_EN_PIN = 26
// // MOTOR_LEFT_P = 8
// // MOTOR_LEFT_N = 11
// // MOTOR_RIGHT_P = 10
// // MOTOR_RIGHT_N = 9

//   public:
//       MotorController() {
//       }
//       bool init();
//     private:
//       double hw_start_sec_;
//       int i2c_fd_;
//   };

//   bool MotorController::init() {
//     char* filename = "/dev/i2c-1";
//     i2c_fd_ = open(filename, O_RDWR);
//     if (i2c_fd_ < 0) {
//       return false;
//     }
//     return true;
//   }
  

  
CallbackReturn TrilosaurusRobotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  trilobot_driver_setup_hardware();
  
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }
  clock_ = rclcpp::Clock();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TrilosaurusRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TrilosaurusRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

   return command_interfaces;
}

CallbackReturn TrilosaurusRobotHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TrilosaurusRobotHardware"), "Activating ...please wait...");
  for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
	rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }
  // TODO(anyone): prepare the robot to receive commands
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
	{
	  hw_positions_[i] = 0;
	  hw_velocities_[i] = 0;
	  hw_commands_[i] = 0;
	}
    }

  last_timestamp_ = clock_.now();
  RCLCPP_INFO(rclcpp::get_logger("TrilosaurusRobotHardware"), "Successfully activated!");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn TrilosaurusRobotHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TrilosaurusRobotHardware"), "Deactivating ...please wait...");
  // TODO(anyone): prepare the robot to stop receiving commands
#if 1
  trilobot_driver_motors_disable();
#else
  for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
        rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }
#endif
  RCLCPP_INFO(rclcpp::get_logger("TrilosaurusRobotHardware"), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type TrilosaurusRobotHardware::read()
{
  current_timestamp = clock_.now();
  rclcpp::Duration dt = current_timestamp - last_timestamp_;  // Control period
  last_timestamp_ = current_timestamp;
#if 0
  // TODO: read robot states
#else
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate wheels's movement: simply integrates
    hw_positions_[i] = hw_positions_[1] + dt.seconds() * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];
  }
#endif
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TrilosaurusRobotHardware::write()
{
  float K = 35.;
  int8_t cmd_left = int(K*hw_commands_[0]);
  int8_t cmd_right = int(K*hw_commands_[1]);
  trilobot_driver_motors_set_speed(0, cmd_left);
  trilobot_driver_motors_set_speed(1, cmd_right);
  return hardware_interface::return_type::OK;
}

}  // namespace trilosaurus_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(trilosaurus_hardware_interface::TrilosaurusRobotHardware, hardware_interface::SystemInterface)
