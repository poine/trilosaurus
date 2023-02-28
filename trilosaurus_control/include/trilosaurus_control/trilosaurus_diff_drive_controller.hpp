// Copyright 2023 Poine
// extending diff drive controller for trilobot
//

#ifndef TRILOSAURUS_CONTROL__TRILOSAURUS_DIFF_DRIVE_CONTROLLER_HPP_
#define TRILOSAURUS_CONTROL__TRILOSAURUS_DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"



#include "diff_drive_controller/diff_drive_controller.hpp"
#include "trilosaurus_control/msg/report.hpp"

namespace trilosaurus_control
{

class TrilosaurusDiffDriveController : public diff_drive_controller::DiffDriveController
{

public:
  //DIFF_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  
private:
  double integ_left, integ_right;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> foo_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> realtime_foo_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<trilosaurus_control::msg::Report>> report_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<trilosaurus_control::msg::Report>> realtime_report_publisher_ = nullptr;

};

}  // trilosaurus_control

#endif // TRILOSAURUS_CONTROL__TRILOSAURUS_DIFF_DRIVE_CONTROLLER_HPP_
