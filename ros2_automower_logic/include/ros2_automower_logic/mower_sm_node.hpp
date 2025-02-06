#ifndef ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_
#define ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int8.hpp"
#include "mower_sm.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros2_automower_control/srv/blade_command.hpp"

namespace ros2_automower_logic
{
class MowerStateMachineNode : public rclcpp::Node
{
public:
  MowerStateMachineNode() : Node("MowerStateMachine") {}

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_map_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_blade_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_blade_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_charge_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mow_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_teleop_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_home_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr e_stop_srv; // TODO need to rethink this. Need lower level controller to trigger estop
  rclcpp::Client<ros2_automower_control::srv::BladeCommand>::SharedPtr blade_command_srv;

  void start_map_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void start_blade_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void stop_blade_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void start_mow_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void start_teleop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void go_home_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void e_stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // TODO rclcpp::Subscription<EVENT>::SharedPtr event_sub; // to trigger finish map event, reach home event, finish charge event
  // TODO rclcpp::Subscription<VOLTAGE>::SharedPtr voltage_sub; // to monitor charge
  // TODO rclcpp::Subscription<CHARGE_CURRENT>::SharedPtr charge_current_sub; // to monitor charge current
  // void event_callback(const Event::SharedPtr msg);
  void location_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  //rclcpp::Publisher<State>::SharedPtr mower_state_pub;


};
}  // namespace ros2_automower_logic
#endif  // ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_
