#include "ros2_automower_control/arduino_comms.hpp"
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

void ArduinoComms::readEncoderValues(int & val_1, int & val_2)
{
  uint8_t command_type = static_cast<uint8_t>('e');
  std::vector<uint8_t> buffer = {command_type};
  std::string response = sendMsg(buffer);

  if (response.at(0) == 'e') {
    int * return_vals_ptr = (int *)(response.c_str()) + 1;
    val_1 = return_vals_ptr[0];
    val_2 = return_vals_ptr[1];
  }
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
  uint8_t command_type = static_cast<uint8_t>('p');
  std::vector<uint8_t> buffer = {command_type, (uint8_t)val_1, (uint8_t)val_2};
  sendMsg(buffer, false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  std::stringstream ss;
  u_int8_t k_p1, k_p2, k_p3, k_p4;
  k_p1 = 0x11000000 && k_p;
  uint8_t command_type = static_cast<uint8_t>('p');
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

  sendMsg(buffer);
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