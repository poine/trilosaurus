// Copyright (c) 2022, poine
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
// Copyright (c) 2021, ros2_control Development Team (example)
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

#ifndef TRILOSAURUS_HARDWARE_INTERFACE__TRILOSAURUS_ROBOT_HARDWARE_HPP_
#define TRILOSAURUS_HARDWARE_INTERFACE__TRILOSAURUS_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "trilosaurus_hardware_interface/visibility_control.h"

namespace trilosaurus_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrilosaurusRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrilosaurusRobotHardware)

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  TRILOSAURUS_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for hardware simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double base_x_, base_y_, base_theta_;
  
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Update loop timing
  rclcpp::Clock clock_;
  rclcpp::Time last_timestamp_;
  rclcpp::Time current_timestamp;

  
};

}  // namespace trilosaurus_hardware_interface

#endif  // TRILOSAURUS_HARDWARE_INTERFACE__TRILOSAURUS_ROBOT_HARDWARE_HPP_
