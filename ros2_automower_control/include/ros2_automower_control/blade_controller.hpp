#ifndef ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_
#define ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_

#include "arduino_comms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_automower_control/msg/blade_status.hpp"
#include "ros2_automower_control/srv/blade_command.hpp"
#include "wheel.hpp"
namespace ros2_automower_control
{
class BladeController : public rclcpp::Node
{
  struct Config
  {
    std::string blade_name = "";
    float loop_rate = 0.0;
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_counts_per_rev = 0;
    float pid_p = 0.0;
    float pid_i = 0.0;
    float pid_d = 0.0;
    float pid_o = 0.0;
  } cfg_;

public:
  BladeController() : Node("BladeController") {}
  void BladeController::setBladeCommand(
    const std::shared_ptr<ros2_automower_control::srv::BladeCommand::Request> request,
    std::shared_ptr<ros2_automower_control::srv::BladeCommand::Response> response);
  void sendBladeCommand();

  void publishBladeStatus();

private:
  ArduinoComms comms_;
  rclcpp::Service<ros2_automower_control::srv::BladeCommand>::SharedPtr blade_command_server;
  rclcpp::Publisher<ros2_automower_control::msg::BladeStatus>::SharedPtr blade_status_publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::TimerBase::SharedPtr command_timer;
  bool blade_active;
  Wheel blade_;
};
}  // namespace ros2_automower_control
#endif  // ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_
