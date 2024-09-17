#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_automower_control/blade_controller.hpp"

namespace ros2_automower_control
{
BladeController::BladeController()
{
    cfg_.blade_name = this->get_parameter("blade_name").get_value<std::string>();
    cfg_.loop_rate = this->get_parameter("loop_rate").get_value<float>();
    cfg_.device = this->get_parameter("device").get_value<std::string>();
    cfg_.baud_rate = this->get_parameter("baud_rate").get_value<int>();
    cfg_.timeout_ms = this->get_parameter("timeout_ms").get_value<int>();
    cfg_.pid_p = this->get_parameter("pid_p").get_value<int>();
    cfg_.pid_i = this->get_parameter("pid_i").get_value<int>();
    cfg_.pid_d = this->get_parameter("pid_d").get_value<int>();
    cfg_.pid_o = this->get_parameter("pid_o").get_value<int>();
    cfg_.enc_counts_per_rev =
        this->get_parameter("enc_counts_per_rev").get_value<int>();

    blade_status_publisher = this->create_publisher<ros2_automower_control::msg::BladeStatus>("blade_status", 1);
    using std::placeholders::_1;
    using std::placeholders::_2;
    blade_command_server = this->create_service<ros2_automower_control::srv::BladeCommand>(
        "blade_command", std::bind(&BladeController::sendBladeCommand, this, _1, _2));
    publish_timer = this->create_wall_timer(
        std::chrono::seconds(0.2),
        std::bind(&BladeController::publishBladeStatus, this));
    command_timer = this->create_wall_timer(
        std::chrono::seconds(0.2),
        std::bind(&BladeController::sendBladeCommand, this));
}


void BladeController::setBladeCommand(
        const std::shared_ptr<ros2_automower_control::srv::BladeCommand::Request> request,
        std::shared_ptr<ros2_automower_control::srv::BladeCommand::Response> response)
    {
        float commanded_vel = request.get()->rads_per_second;
        RCLCPP_INFO(this->get_logger(), "Received blade command");
        blade_.cmd = commanded_vel;
       
        RCLCPP_INFO(rclcpp::get_logger("BladeController"), "Blade command set");
    }


void BladeController::sendBladeCommand()
    {
        float commanded_vel = request.get()->rads_per_second;
        RCLCPP_INFO(this->get_logger(), "Received blade command");
        int motor_counts_per_loop = blade_.cmd / blade_.rads_per_count / cfg_.loop_rate;
        std::pair<bool, std::string>response = comms_.setMotorValues(motor_counts_per_loop);
        if (response.first == true){
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Joints successfully written!");

        return hardware_interface::return_type::OK;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("MotorInterface"), response.second);
            return hardware_interface::return_type::ERROR;
        };
    }

void BladeController::publishBladeStatus()
{
    auto msg = ros2_automower_control::msg::BladeStatus();
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
