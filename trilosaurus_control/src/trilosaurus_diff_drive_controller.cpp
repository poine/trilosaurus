#include "trilosaurus_control/trilosaurus_diff_drive_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#define DEFAULT_DEBUG_OUT_TOPIC "/trilosaurus_controller/debug"

namespace trilosaurus_control
{

controller_interface::CallbackReturn TrilosaurusDiffDriveController::on_configure(const rclcpp_lifecycle::State &_s) {
  auto _r = DiffDriveController::on_configure(_s);
  // foo_publisher_ =
  //   get_node()->create_publisher<std_msgs::msg::String>(DEFAULT_DEBUG_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
  // realtime_foo_publisher_ =
  //   std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(foo_publisher_);

  report_publisher_  =
    get_node()->create_publisher<trilosaurus_control::msg::Report>(DEFAULT_DEBUG_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_report_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<trilosaurus_control::msg::Report>>(report_publisher_);
  
  return _r;
}
   
controller_interface::return_type TrilosaurusDiffDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period) {

  
  // run the original diff drive controller
  auto _r = DiffDriveController::update(time, period);
  // make sure we're initialized and running
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return controller_interface::return_type::OK;
  }

  
  auto logger = get_node()->get_logger();
  auto clock = get_node()->get_clock();
  //RCLCPP_INFO_THROTTLE(logger, *clock, 1000, "Foo");
  //RCLCPP_INFO(logger, "Foo");

  // Dimensions for kinematics:
  const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
  const double left_wheel_radius = params_.left_wheel_radius_multiplier * params_.wheel_radius;
  const double right_wheel_radius = params_.right_wheel_radius_multiplier * params_.wheel_radius;

  // measurement (encoders)
  //auto meas_pos_left = registered_left_wheel_handles_[0].feedback.get().get_value();
  //auto meas_pos_right = registered_right_wheel_handles_[0].feedback.get().get_value();
  double meas_linear = odometry_.getLinear();
  double meas_angular = odometry_.getAngular();
  double meas_left  = (meas_linear - meas_angular * wheel_separation / 2.0) / left_wheel_radius;
  double meas_right = (meas_linear + meas_angular * wheel_separation / 2.0) / right_wheel_radius;
  
  // diff drive controller output
  //auto ul = registered_left_wheel_handles_[0].velocity.get().get_value();
  //auto ur = registered_right_wheel_handles_[0].velocity.get().get_value();

  std::shared_ptr<geometry_msgs::msg::TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);
  const auto age_of_last_command = time - last_command_msg->header.stamp;
  if (age_of_last_command > cmd_vel_timeout_) // diff drive will have timeouted and zeroed command
    return _r;
  geometry_msgs::msg::TwistStamped command = *last_command_msg;
  double & sp_linear = command.twist.linear.x;
  double & sp_angular = command.twist.angular.z;
  double sp_left = (sp_linear - sp_angular * wheel_separation / 2.0) / left_wheel_radius;
  double sp_right = (sp_linear + sp_angular * wheel_separation / 2.0) / right_wheel_radius;

  double err_left = meas_left - sp_left, err_right = meas_right - sp_right; 
  const double sat_err = 5.; // rad/s
  err_left  = std::clamp(err_left,  -sat_err, sat_err);
  err_right  = std::clamp(err_right,  -sat_err, sat_err);

  double Kp = -4., Ki = -0.050;
  //double Kp = 0., Ki = -0.075;
  double sat_integ = -50/Ki;
  double dts = double(period.seconds()) + 1e-9*double(period.nanoseconds());
  integ_left += err_left/dts;
  integ_right += err_right/dts;
  integ_left = std::clamp(integ_left, -sat_integ, sat_integ);
  integ_right = std::clamp(integ_right, -sat_integ, sat_integ);
  double min_sp = 0.05;
  if  (sp_left < min_sp && sp_left > -min_sp  ) {integ_left = 0;}
  if  (sp_right < min_sp && sp_right > -min_sp  ) {integ_right = 0;}
    
  double cloop_left  = Kp*err_left  + Ki*integ_left;
  double cloop_right = Kp*err_right + Ki*integ_right;

  double oloop_left = 0.; double oloop_right = 0.;
  
  //int8_t cmd_left = int8_t(cloop_left);
  //int8_t cmd_right = int8_t(cloop_right);
  double cmd_left = cloop_left + oloop_left;
  double cmd_right = cloop_right + oloop_right;
  //int8_t sat_cmd = 100;
  double sat_cmd = 100.;
  cmd_left = std::clamp(cmd_left,   -sat_cmd, sat_cmd);
  cmd_right = std::clamp(cmd_right, -sat_cmd, sat_cmd);
  registered_left_wheel_handles_[0].velocity.get().set_value(cmd_left);
  registered_right_wheel_handles_[0].velocity.get().set_value(cmd_right);
  //RCLCPP_INFO_THROTTLE(logger, *clock, 1000, "lin/ang measured: %.02f %.02f  setpoint: %.02f %.02f",meas_linear, meas_angular, sp_linear, sp_angular);
  //RCLCPP_INFO_THROTTLE(logger, *clock, 1000, "left/right measured: %.02f %.02f  setpoint: %.02f %.02f", meas_left, meas_right, sp_left, sp_right);
  //RCLCPP_INFO_THROTTLE(logger, *clock, 500, "left/right err : %.02f %.02f  integ: %.0f %.0f (/ %.0f)  U:%.1f %.1f", err_left, err_right, integ_left, integ_right, sat_integ, cmd_left, cmd_right);
  //RCLCPP_INFO_THROTTLE(logger, *clock, 1000, "left/right output : %.02f %.02f cl %f %f U:%.1f %.1f", ul, ur, cloop_left, cloop_right, cmd_left, cmd_right);
  //RCLCPP_INFO_THROTTLE(logger, *clock, 500, "left/right cl %.1ff %.1f U:%.1f %.1f", cloop_left, cloop_right, cmd_left, cmd_right);
  //int8_t _cml = int8_t(cmd_left), _cmr = int8_t(cmd_right);
  //registered_left_wheel_handles_[0].velocity.get().set_value(_cml);
  //registered_right_wheel_handles_[0].velocity.get().set_value(_cmr);

  if (realtime_report_publisher_->trylock())
    {
      auto & _message = realtime_report_publisher_->msg_;
      _message.header.stamp = get_node()->get_clock()->now();
      _message.meas_x = meas_linear;
      _message.meas_z = meas_angular;
      _message.sp_x = sp_linear;
      _message.sp_z = sp_angular;
      _message.meas_left = meas_left;
      _message.meas_right = meas_right;
      _message.sp_left = sp_left;
      _message.sp_right = sp_right;
      _message.cmd_left = int(cmd_left);
      _message.cmd_right = int(cmd_right);
      realtime_report_publisher_->unlockAndPublish();
    }






  
  return _r; //controller_interface::return_type::OK;
}


}





#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  trilosaurus_control::TrilosaurusDiffDriveController, controller_interface::ControllerInterface)
