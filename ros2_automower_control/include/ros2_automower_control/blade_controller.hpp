#ifndef ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_
#define ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_

#include "arduino_comms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wheel.hpp"
namespace ros2_automower_conrtol
{
class BladeController : public rclcpp::Node
{
public:
  BladeController() : Node("BladeController") {}

  void sendBladeCommand(
    const std::shared_ptr<Blade_Command::Request> request,
    std::shared_ptr<Blade_Command::Response> response);

  void publishBladeStatus();

private:
  ArduinoComms comms_;
  rclcpp::Service<Blade_Command>::SharedPtr blade_command_server;
  rclcpp::Publisher<Blade_Status>::SharedPtr blade_status_publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;
  bool blade_active;
  Wheel blade_;
};
}  // namespace ros2_automower_conrtol
#endif  // ROS2_AUTOMOWER_CONTROL__BLADE_CONTROLLER_HPP_
