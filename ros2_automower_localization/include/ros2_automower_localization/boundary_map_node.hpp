#ifndef ROS2_AUTOMOWER_LOCALIZATION__BOUNDARY_MAP_NODE_HPP_
#define ROS2_AUTOMOWER_LOCALIZATION__BOUNDARY_MAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ros2_automower_logic/msg/event.hpp"
namespace ros2_automower_localization
{
class BoundaryMapNode : public rclcpp::Node
{
public:
    BoundaryMapNode() : Node("boundary_map")
    {

    }



private:
    geometry_msgs::msg::Polygon boundary_map;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unset_map_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_map_service;
    rclcpp::Publisher<ros2_automower_logic::msg::Event>::SharedPtr event_publisher;

};
}  // namespace ros2_automower_localization
#endif  // ROS2_AUTOMOWER_LOCALIZATION__BOUNDARY_MAP_NODE_HPP_
