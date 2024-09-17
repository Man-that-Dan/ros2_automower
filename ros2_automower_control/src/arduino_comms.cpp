#include "ros2_automower_control/arduino_comms.hpp"

#include "ros2_automower_control/command_types.h"
// #include <ros/console.h>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#define FLOAT_SIZE 4
void ArduinoComms::setup(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  LibSerial::BaudRate b_rate;
  switch (baud_rate) {
    case 9600:
      b_rate = LibSerial::BaudRate::BAUD_9600;
      break;
    case 115200:
      b_rate = LibSerial::BaudRate::BAUD_115200;
      break;
    case 19200:
      b_rate = LibSerial::BaudRate::BAUD_19200;
      break;
    case 38400:
      b_rate = LibSerial::BaudRate::BAUD_38400;
      break;
    default:
      b_rate = LibSerial::BaudRate::BAUD_9600;
      break;
  }
  if (serial_conn_.IsOpen()) {
    serial_conn_.Close();
  }
  serial_conn_.Open(serial_device);
  serial_conn_.SetBaudRate(b_rate);
  serial_conn_.SetVTime(timeout_ms * 100);
}

void ArduinoComms::sendEmptyMsg() { std::string response = sendMsg({0x00}); }

std::pair<bool, std::string> ArduinoComms::readEncoderValues(int & val_1, int & val_2)
{
  bool success;
  std::string message;
  if (serial_conn_.IsOpen()) {
    uint8_t command_type = static_cast<uint8_t>(command_type::FEEDBACK);
    std::vector<uint8_t> buffer = {command_type};
    std::string response = sendMsg(buffer);

    if (response.at(0) == command_type::FEEDBACK) {
      int * return_vals_ptr = (int *)(response.c_str()) + 1;
      val_1 = return_vals_ptr[0];
      val_2 = return_vals_ptr[1];
      success = true;
      message = "Successfully retrieved encoder values";
      return std::pair<bool, std::string>(success, message);
    } else {
      success = false;
      message = "Failed to retrieve encoder values";
      return std::pair<bool, std::string>(success, message);
    }
  } else {
    success = false;
    message = "Serial Connection Closed";
    return std::pair<bool, std::string>(success, message);
  }
}

std::pair<bool, std::string> ArduinoComms::readBladeEncoderValue(int & val)
{
  bool success;
  std::string message;
  if (serial_conn_.IsOpen()) {
    uint8_t command_type = static_cast<uint8_t>(command_type::FEEDBACK);
    std::vector<uint8_t> buffer = {command_type};
    std::string response = sendMsg(buffer);

    if (response.at(0) == command_type::FEEDBACK) {
      int * return_vals_ptr = (int *)(response.c_str()) + 1;
      val = return_vals_ptr[0];
      success = true;
      message = "Successfully retrieved blade encoder value";
      return std::pair<bool, std::string>(success, message);
    } else {
      success = false;
      message = "Failed to retrieve blade encoder value";
      return std::pair<bool, std::string>(success, message);
    }
  } else {
    success = false;
    message = "Serial Connection Closed";
    return std::pair<bool, std::string>(success, message);
  }
}

std::pair<bool, std::string> ArduinoComms::setBladeMotorValue(int val)
{
  bool success;
  std::string message;
  if (serial_conn_.IsOpen()) {
    uint8_t command_type = static_cast<uint8_t>(command_type::COMMAND);
    std::vector<uint8_t> buffer = {command_type, (uint8_t)val};
    std::string response = sendMsg(buffer, false);
    bool success;
    std::string message;
    if (response.at(0) == command_type::COMMAND) {
      int * returned_values = (int *)response.c_str() + 1;
      int val_returned;
      val_returned = returned_values[0];
      if (val == val_returned) {
        success = true;
        message = "Successfully set blade motor command";
        return std::pair<bool, std::string>(success, message);
      } else {
        success = false;
        std::stringstream message_stream;
        message_stream << "Returned blade command does not match: ";
        message_stream << " val : " << val_returned;
        message_stream << std::endl;
        return std::pair<bool, std::string>(success, message.c_str());
      };
    } else {
      success = false;
      message = "Set Blade Motor Speed command not aknowledged";
      return std::pair<bool, std::string>(success, message);
    };
  } else {
    success = false;
    message = "Serial Connection Closed";
    return std::pair<bool, std::string>(success, message);
  };
}

std::pair<bool, std::string> ArduinoComms::setMotorValues(int val_1, int val_2)
{
  bool success;
  std::string message;
  if (serial_conn_.IsOpen()) {
    uint8_t command_type = static_cast<uint8_t>(command_type::COMMAND);
    std::vector<uint8_t> buffer = {command_type, (uint8_t)val_1, (uint8_t)val_2};
    std::string response = sendMsg(buffer, false);
    bool success;
    std::string message;
    if (response.at(0) == command_type::COMMAND) {
      int * returned_values = (int *)response.c_str() + 1;
      int val_1_returned, val_2_returned;
      val_1_returned = returned_values[0];
      val_2_returned = returned_values[1];
      if ((val_1 == val_1_returned) && (val_2 == val_2_returned)) {
        success = true;
        message = "Successfully set motor commands";
        return std::pair<bool, std::string>(success, message);
      } else {
        success = false;
        std::stringstream message_stream;
        message_stream << "Returned commands do not match: ";
        message_stream << " val_1 : " << val_1_returned;
        message_stream << " val_2 : " << val_2_returned << std::endl;
        return std::pair<bool, std::string>(success, message.c_str());
      };
    } else {
      success = false;
      message = "Set Motor Speed command not aknowledged";
      return std::pair<bool, std::string>(success, message);
    };
  } else {
    success = false;
    message = "Serial Connection Closed";
    return std::pair<bool, std::string>(success, message);
  };
}

std::pair<bool, std::string> ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  bool success;
  std::string message;
  if (serial_conn_.IsOpen()) {
    uint8_t command_type = static_cast<uint8_t>(command_type::GAINS);
    std::vector<uint8_t> buffer = {command_type};

    uint8_t * k_p_ptr = (uint8_t *)&k_p;
    for (int i = 0; i < FLOAT_SIZE; i++) {
      buffer.emplace_back(k_p_ptr[i]);
    };

    uint8_t * k_i_ptr = (uint8_t *)&k_i;
    for (int i = 0; i < FLOAT_SIZE; i++) {
      buffer.emplace_back(k_i_ptr[i]);
    };

    uint8_t * k_d_ptr = (uint8_t *)&k_d;
    for (int i = 0; i < FLOAT_SIZE; i++) {
      buffer.emplace_back(k_d_ptr[i]);
    };

    uint8_t * k_o_ptr = (uint8_t *)&k_o;
    for (int i = 0; i < FLOAT_SIZE; i++) {
      buffer.emplace_back(k_o_ptr[i]);
    };

    bool success;
    std::string message;
    std::string response = sendMsg(buffer, false);
    if (response.at(0) == command_type::GAINS) {
      float * returned_values = (float *)response.c_str() + 1;
      float k_p_returned, k_i_returned, k_d_returned, k_o_returned;
      k_p_returned = returned_values[0];
      k_i_returned = returned_values[1];
      k_d_returned = returned_values[2];
      k_o_returned = returned_values[3];
      if (
        (k_p_returned == k_p) && (k_i_returned == k_i) && (k_d_returned == k_d) &&
        (k_o_returned == k_o)) {
        success = true;
        message = "Successfully set gains";
        return std::pair<bool, std::string>(success, message);
      } else {
        success = false;
        std::stringstream message_stream;
        message_stream << "Returned gains do not match: ";
        message_stream << " k_p : " << k_p_returned;
        message_stream << " k_i : " << k_i_returned;
        message_stream << " k_d : " << k_d_returned;
        message_stream << " k_o : " << k_o_returned << std::endl;
        return std::pair<bool, std::string>(success, message.c_str());
      };
    } else {
      success = false;
      message = "Set Gains command not aknowledged";
      return std::pair<bool, std::string>(success, message);
    };
  } else {
    success = false;
    message = "Serial Connection Closed";
    return std::pair<bool, std::string>(success, message);
  };
}

std::string ArduinoComms::sendMsg(const std::vector<uint8_t> & msg_to_send, bool print_output)
{
  serial_conn_.Write(msg_to_send);
  std::string response;
  serial_conn_.ReadLine(response);

  if (print_output) {
    // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
    // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
  }

  return response;
}

void ArduinoComms::disconnect() { serial_conn_.Close(); }