#ifndef ROS2_AUTOMOWER_LOGIC__MOWER_EVENTS_HPP
#define ROS2_AUTOMOWER_LOGIC__MOWER_EVENTS_HPP

#include <string>

#include "tinyfsm.hpp"

struct StartMapEvent : tinyfsm::Event
{
  std::pair<float, float> start_coordinates;
};

struct FinishMapEvent : tinyfsm::Event
{
  bool success;
};

struct ReachedHomeEvent : tinyfsm::Event
{
};

struct EmergencyStopEvent : tinyfsm::Event
{
  std::string message;
};

struct LowBatteryEvent : tinyfsm::Event
{
  float voltage;
};

struct FinishedMowEvent : tinyfsm::Event
{
  bool success;
};

struct StartMowEvent : tinyfsm::Event
{
};

struct StartTeleopEvent : tinyfsm::Event
{
};

struct CancelEvent : tinyfsm::Event
{
};

struct HomingEvent : tinyfsm::Event
{
};

struct FinishChargeEvent : tinyfsm::Event
{
};

#endif // !ROS2_AUTOMOWER_LOGIC__MOWER_EVENTS_HPP