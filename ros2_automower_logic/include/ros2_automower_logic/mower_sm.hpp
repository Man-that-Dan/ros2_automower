#include <cmath>

#include "mower_events.hpp"
#include "tinyfsm.hpp"
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

  virtual void entry(void){};
  virtual void exit(void){};
  void stopRobot();
  std::pair<double, double> getHomeCoordinates(){
    return home_coordinates;
  };
  std::pair<double, double> getCurrentCoordinates(){
    return current_coordinates;
  };
  void setCurrentCoordinates(std::pair<double, double> coordinates){
    current_coordinates = coordinates;
  };
  void setHomeCoordinates(std::pair<double, double> coordinates){
    home_coordinates = coordinates;
  };
  double getDoubleParameter(std::string parameter);
  void setNavigationTarget(std::pair<double, double> coordinates);

private:
  static std::pair<double,double> current_coordinates;
  static std::pair<double,double> home_coordinates;
};

class Idle : public MowerState
{
  void entry() override;
  void react(StartMapEvent const &) override;
  void react(LowBatteryEvent const &) override;
  void react(StartTeleopEvent const &) override;
  void react(StartMowEvent const &) override;
};

class Mapping : public MowerState
{
  void entry() override
  {
    unsetCurrentMap();
    startMapper();
  };
  void react(FinishMapEvent const &) override{

  };
  void react(LowBatteryEvent const &) override;
  void react(StartTeleopEvent const &) override;
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
  void react(ReachedHomeEvent const &) override;
  void unsetCurrentNavigationCourse();
};

class Charging : public MowerState
{
  void entry() override { startChargeMonitor(); };
  void exit() override
  {
    stopChargeMonitor();
    returnToPreHomingState();
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
};