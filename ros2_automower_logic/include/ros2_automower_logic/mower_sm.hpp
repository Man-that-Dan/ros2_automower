/***
 * Simple State Machine for Autonomous mower. Transitions, and actions declared here.
 * Implementation defined in specific framework used. In this project, ROS2 is used
 * so the specific robot communication functions are implemented in a ros2 node cpp file.
 * This allows the implementation and frameworks to be switched without changing this core logic.
*/
#ifndef ROS2_AUTOMOWER_LOGIC__MOWER_STATE_HPP
#define ROS2_AUTOMOWER_LOGIC__MOWER_STATE_HPP
#include <cmath>
#include <variant>

#include "mower_events.hpp"
#include "tinyfsm.hpp"
using PreHomingState = std::variant<Idle, Mapping, Mowing>;
class MowerState : public tinyfsm::Fsm<MowerState>
{
public:
  MowerState() {}
  /* default reaction for unhandled events */
  void react(tinyfsm::Event const &){};

  virtual void react(FinishedMowEvent const &);
  virtual void react(FinishMapEvent const &);
  virtual void react(ReachedHomeEvent const &);
  void react(EmergencyStopEvent const &);
  virtual void react(LowBatteryEvent const &);
  virtual void react(StartMapEvent const &);
  virtual void react(StartTeleopEvent const &);
  virtual void react(StartMowEvent const &);
  virtual void react(CancelEvent const &);
  virtual void react(HomingEvent const &);

  virtual void entry(void){};
  virtual void exit(void){};
  void stopRobot();
  std::pair<double, double> getHomeCoordinates() { return home_coordinates; };
  std::pair<double, double> getCurrentCoordinates() { return current_coordinates; };
  void setCurrentCoordinates(std::pair<double, double> coordinates)
  {
    current_coordinates = coordinates;
  };
  void setHomeCoordinates(std::pair<double, double> coordinates)
  {
    home_coordinates = coordinates;
  };
  double getDoubleParameter(std::string parameter);
  void setNavigationTarget(std::pair<double, double> coordinates);
  static PreHomingState pre_homing_state;
private:
  static std::pair<double, double> current_coordinates;
  static std::pair<double, double> home_coordinates;
  
};

class Idle : public MowerState
{
  void react(StartMapEvent const &) override {
    transit<Mapping>();
  };
  void react(LowBatteryEvent const &) override {
    pre_homing_state = *current_state_ptr;
    transit<Homing>();
  };
  void react(StartMowEvent const &) override {
    transit<Mowing>();
  };
  void react(HomingEvent const &) override {
    transit<Homing>();
  };
};

class Mapping : public MowerState
{
  void entry() override
  {
    unsetCurrentMap();
    startMapper();
  };
  void react(FinishMapEvent const &) override {
    transit<Idle>();
  };
  void react(LowBatteryEvent const &) override {
    pre_homing_state = *current_state_ptr;
    transit<Homing>();
  };
  void react(CancelEvent const &) override {
    transit<Idle>();
  };
  void react(HomingEvent const &) override {
    transit<Homing>();
  };

  void exit() override
  {
    stopRobot();
    stopMapper();
  };
  void unsetCurrentMap();
  void startMapper();
  void stopMapper();
};

class Homing : public MowerState
{
  void entry() override
  {
    unsetCurrentNavigationCourse();
    std::pair<double, double> home_location = getHomeCoordinates();
    setNavigationTarget(home_location);
  };
  void exit() override
  {
    unsetCurrentNavigationCourse();
    stopRobot();
  }
  void react(ReachedHomeEvent const &) override {
    transit<Charging>();
  };
  void react(CancelEvent const &) override {
    transit<Idle>();
  };
  void unsetCurrentNavigationCourse();
};

class Charging : public MowerState
{
  void entry() override { startChargeMonitor(); };
  void exit() override
  {
    stopChargeMonitor();
  };
  void react(StartTeleopEvent const &) {
    transit<Idle>();
  };
  void react(FinishChargeEvent const &) {
    transit<pre_homing_state.type()>();
  };
  void startChargeMonitor();
  void stopChargeMonitor();
  void returnToPreHomingState();
};

class Mowing : public MowerState
{
  void entry() override
  {
    std::pair<double, double> home_location = getHomeCoordinates();
    std::pair<double, double> current_location = getCurrentCoordinates();
    double distance = std::hypot(
      current_location.first - home_location.first, current_location.second - home_location.second);
    double min_home_distance = getDoubleParameter("min_home_distance");
    bool success = true;
    if (distance < min_home_distance) {
      // Back up out of home
      success = driveInReverse(current_location, distance);
    }
    if (success) {
      bool have_plan = checkIfMowingPlan();
      bool plan_success = true;
      if (!have_plan) {
        plan_success = createMowingPlan();
      }
      if (plan_success) {
        startBlade();
        startMowing();
      }
    }
  };
  void react(LowBatteryEvent const &) override {
    pre_homing_state = *current_state_ptr;
    transit<Homing>();
  };
  void react(CancelEvent const &) override {
    transit<Idle>();
  };
  void exit() override { stopRobot(); };
  bool driveInReverse(std::pair<double, double> current_location, double distance);
  void startBlade();
  bool checkIfMowingPlan();
  bool createMowingPlan();
  void startMowing();
};

class EmergencyStop : public MowerState
{
  void entry() override { stopRobot(); };
  void exit() override { transit<Idle>(); };
};

FSM_INITIAL_STATE(MowerState, Idle);

using fsm_list = tinyfsm::FsmList<MowerState>;

template<typename E>
void send_event(E const & Event)
{
  fsm_list::template dispatch<E>(event);
}

#endif // !ROS2_AUTOMOWER_LOGIC__MOWER_STATE_HPP