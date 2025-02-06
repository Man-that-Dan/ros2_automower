#include "../include/ros2_automower_logic/mower_sm_node.hpp"

#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "../include/ros2_automower_logic/simple_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"

static auto state_machine_function_node = rclcpp::Node::make_shared("StateMachineFunctionNode");

double MowerState::getDoubleParameter(std::string parameter)
{
  static auto params_client = std::make_shared<rclcpp::SyncParametersClient>(
    state_machine_function_node, "MowerStateMachine");
  double value = params_client->get_parameter(parameter, -1.0);
  return value;
};

void MowerState::stopRobot()
{
  //TODO, correct service name
  static auto stop_robot_client =
    state_machine_function_node->create_client<std_srvs::srv::Trigger>("/gotoPose");
  while (!stop_robot_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        state_machine_function_node->get_logger(), "Interruped while waiting for the server.");
      return;
    }
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "Server not available, waiting again...");
  }
  using std::placeholders::_1;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  RCLCPP_INFO(state_machine_function_node->get_logger(), "Sending request");
  auto future = stop_robot_client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(3));  //not spinning here!
  if (status == std::future_status::ready) {
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "stop robot srv response is %s",
      future.get()->message.c_str());
    bool success = future.get()->success;
    if (!success) {
      RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot failed");
    }
  } else {
    RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot srv future wait failed");
  }
};
void MowerState::setNavigationTarget(std::pair<double, double> coordinates)
{
  static auto set_nav_client =
    std::make_shared<SimpleActionClient<nav2_msgs::action::NavigateToPose>>(
      "navigateToPoseClient", "/navigateToPose", 1);

  using std::placeholders::_1;
  using std::placeholders::_2;
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position.x = coordinates.first;
  goal.pose.pose.position.y = coordinates.second;
  bool success = set_nav_client->send_goal(goal);
  if (success) {
    while (!set_nav_client->has_result()) {
      if (set_nav_client->has_feedback()) {
        auto feedback = set_nav_client->get_feedback();
        RCLCPP_INFO(set_nav_client->get_logger(), "Feedback received");
      }
      rclcpp::spin_some(set_nav_client);
    }
    auto result = set_nav_client->get_result();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(set_nav_client->get_logger(), "Could not navigate to target");
    };
  }
};

//TODO implement
void Mapping::unsetCurrentMap(){
};
void Mapping::startMapper(){};
void Mapping::stopMapper(){};
void Homing::unsetCurrentNavigationCourse(){};
void Charging::startChargeMonitor(){};
void Charging::stopChargeMonitor(){};
void Charging::returnToPreHomingState(){};
bool Mowing::driveInReverse(std::pair<double, double> current_location, double distance){};
void MowerState::startBlade(){
  //TODO, correct service name
  static auto blade_client =
    state_machine_function_node->create_client<ros2_automower_control::srv::BladeCommand>("/blade_command");
  while (!blade_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        state_machine_function_node->get_logger(), "Interruped while waiting for the server.");
      return;
    }
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "Server not available, waiting again...");
  }
  auto request = std::make_shared<ros2_automower_control::srv::BladeCommand::Request>();
  request->rads_per_second = 10.0;
  RCLCPP_INFO(state_machine_function_node->get_logger(), "Sending request");
  auto future = blade_client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(3));  //not spinning here!
  if (status == std::future_status::ready) {
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "blade command srv response is %d",
      future.get()->success);
    bool success = future.get()->success;
    if (!success) {
      RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot failed");
    }
  } else {
    RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot srv future wait failed");
  }
};

void MowerState::stopBlade(){
  static auto blade_client =
    state_machine_function_node->create_client<ros2_automower_control::srv::BladeCommand>("/blade_command");
  while (!blade_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        state_machine_function_node->get_logger(), "Interruped while waiting for the server.");
      return;
    }
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "Server not available, waiting again...");
  }
  auto request = std::make_shared<ros2_automower_control::srv::BladeCommand::Request>();
  request->rads_per_second = 0.0;
  RCLCPP_INFO(state_machine_function_node->get_logger(), "Sending request");
  auto future = blade_client->async_send_request(request);
  auto status = future.wait_for(std::chrono::seconds(3));  //not spinning here!
  if (status == std::future_status::ready) {
    RCLCPP_INFO(
      state_machine_function_node->get_logger(), "blade command srv response is %d",
      future.get()->success);
    bool success = future.get()->success;
    if (!success) {
      RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot failed");
    }
  } else {
    RCLCPP_ERROR(state_machine_function_node->get_logger(), "stop robot srv future wait failed");
  }
};
bool Mowing::checkIfMowingPlan(){};
bool Mowing::createMowingPlan(){};
void Mowing::startMowing(){};

namespace ros2_automower_logic {
  MowerStateMachineNode::MowerStateMachineNode(){
    using std::placeholders::_1;
    using std::placeholders::_2;
    start_mow_srv = this->create_service<std_srvs::srv::Trigger>(
      "/start_mow", std::bind(&MowerStateMachineNode::start_mow_callback, this, _1, _2));
    start_map_srv = this->create_service<std_srvs::srv::Trigger>(
      "/start_map", std::bind(&MowerStateMachineNode::start_map_callback, this, _1, _2));
    start_blade_srv = this->create_service<std_srvs::srv::Trigger>(
      "/start_blade", std::bind(&MowerStateMachineNode::start_blade_callback, this, _1, _2));
    stop_blade_srv = this->create_service<std_srvs::srv::Trigger>(
      "/stop_blade", std::bind(&MowerStateMachineNode::stop_blade_callback, this, _1, _2));
    start_teleop_srv = this->create_service<std_srvs::srv::Trigger>(
      "/start_teleop", std::bind(&MowerStateMachineNode::start_teleop_callback, this, _1, _2));
    go_home_srv = this->create_service<std_srvs::srv::Trigger>(
      "/go_home", std::bind(&MowerStateMachineNode::go_home_callback, this, _1, _2));
    cancel_srv = this->create_service<std_srvs::srv::Trigger>(
      "/cancel", std::bind(&MowerStateMachineNode::cancel_callback, this, _1, _2));
    e_stop_srv = this->create_service<std_srvs::srv::Trigger>(
      "/e_stop", std::bind(&MowerStateMachineNode::e_stop_callback, this, _1, _2));
    // event_sub = this->create_subscription<EVENT>(
    //   "/mower_events", 10, std::bind(&MowerStateMachineNode::event_callback, this, _1));
    // TODO set home coords
    using std::placeholders::_1;
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/location", 10, std::bind(&MowerStateMachineNode::location_callback, this, _1));
  };

  void MowerStateMachineNode::start_map_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      StartMapEvent event;
      send_event(event);
};

void MowerStateMachineNode::start_blade_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      StartBladeEvent event;
      send_event(event);
};

void MowerStateMachineNode::stop_blade_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      StopBladeEvent event;
      send_event(event);
};

  void MowerStateMachineNode::start_mow_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      StartMowEvent event;
      send_event(event);
};

  void MowerStateMachineNode::start_teleop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      StartTeleopEvent event;
      send_event(event);
};

  void MowerStateMachineNode::cancel_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      CancelEvent event;
      send_event(event);
};

  void MowerStateMachineNode::go_home_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      HomingEvent event;
      send_event(event);
};

  void MowerStateMachineNode::e_stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      EmergencyStopEvent event;
      send_event(event);
};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_automower_logic::MowerStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
