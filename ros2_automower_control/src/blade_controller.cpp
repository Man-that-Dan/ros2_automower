#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_automower_control/blade_controller.hpp"

namespace ros2_automower_control
{
BladeController::BladeController()
{
    blade_status_publisher = this->create_publisher<Blade_Status>("blade_status", 1);
    using std::placeholders::_1;
    using std::placeholders::_2;
    blade_command_server = this->create_service<Blade_Command>(
        "blade_command", std::bind(&BladeController::sendBladeCommand, this, _1, _2));
    publish_timer = this->create_wall_timer(
        std::chrono::seconds(0.2),
        std::bind(&BladeController::publishBladeStatus, this));
}
void BladeController::sendBladeCommand(
        const std::shared_ptr<Blade_Command::Request> request,
        std::shared_ptr<Blade_Command::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request");
    }

void BladeController::publishBladeStatus()
{
    auto msg = Blade_Status();
    msg.active = blade_active;
    auto publish_period = this->get_parameter("publish_period").get_value<double>();
    double delta_seconds = publish_period;
    int throwaway;
    std::pair<bool, std::string>response = comms_.readEncoderValues(blade_.enc, throwaway);
    float pos_prev = blade_.pos;
    blade_.pos = blade_.calcEncAngle();
    blade_.vel = (blade_.pos - pos_prev) / delta_seconds;
    msg.rads_per_second = blade_.vel;
    RCLCPP_INFO(this->get_logger(), "Publishing message");
    blade_status_publisher->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Timer event");
}
}  // namespace ros2_automower_control


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BladeController>());
    rclcpp::shutdown();
    return 0;
}
