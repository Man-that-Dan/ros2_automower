#include "../include/ros2_automower_logic/mower_sm_node.hpp"
#include "../include/ros2_automower_logic/simple_action_client.hpp"

#include <memory>
#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
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
void MowerState::setNavigationTarget(std::pair<double, double> coordinates){

    static auto set_nav_client = std::make_shared<SimpleActionClient<nav2_msgs::action::NavigateToPose>>("navigateToPoseClient",
        "/navigateToPose", 1);
    
    
    using std::placeholders::_1;
    using std::placeholders::_2;
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position.x = coordinates.first;
    goal.pose.pose.position.y = coordinates.second;
    bool success = set_nav_client->send_goal(goal);
    if(success){
        while(!set_nav_client->has_result()){
            if(set_nav_client->has_feedback()){
                auto feedback = set_nav_client->get_feedback();
                RCLCPP_INFO(set_nav_client->get_logger(), "Feedback received"); 
            }
            rclcpp::spin_some(set_nav_client);
        }
        auto result = set_nav_client->get_result();
        if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_ERROR(set_nav_client->get_logger(), "Could not navigate to target");
        };
    }
 
};

//TODO implement
void Mapping::unsetCurrentMap(){ };
void Mapping::startMapper(){};
void Mapping::stopMapper(){};
void Homing::unsetCurrentNavigationCourse(){ };
void Charging::startChargeMonitor(){ };
void Charging::stopChargeMonitor(){};
void Charging::returnToPreHomingState(){};
bool Mowing::driveInReverse(std::pair<double, double> current_location, double distance){ };
void Mowing::startBlade(){};
bool Mowing::checkIfMowingPlan(){};
bool Mowing::createMowingPlan(){};
void Mowing::startMowing(){};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::spin
  //rclcpp::shutdown();
  return 0;
}
