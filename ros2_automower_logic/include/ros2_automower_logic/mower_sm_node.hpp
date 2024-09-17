#ifndef ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_
#define ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mower_sm.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace ros2_automower_logic
{
class MowerStateMachineNode : public rclcpp::Node
{
public:
    MowerStateMachineNode() : Node("MowerStateMachine")
    {

    }

private:

};
}  // namespace ros2_automower_logic
#endif  // ROS2_AUTOMOWER_LOGIC__MOWER_SM_NODE_HPP_
