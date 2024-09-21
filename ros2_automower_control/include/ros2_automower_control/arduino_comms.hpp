// taken from : https://github.com/joshnewans/diffdrive_arduino/blob/main/include/diffdrive_arduino/arduino_comms.h
// https://www.youtube.com/watch?v=J02jEKawE5U
// BSD 3-Clause License

// Copyright (c) 2020, Josh Newans
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <libserial/SerialPort.h>

#include <cstring>

class ArduinoComms
{
public:
  ArduinoComms() {}

  ArduinoComms(const std::string & serial_device, LibSerial::BaudRate baud_rate)
  : serial_conn_(
      serial_device, baud_rate, LibSerial::CharacterSize::CHAR_SIZE_DEFAULT,
      LibSerial::FlowControl::FLOW_CONTROL_DEFAULT, LibSerial::Parity::PARITY_DEFAULT,
      LibSerial::StopBits::STOP_BITS_DEFAULT)
  {
  }

  void setup(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  std::pair<bool, std::string> readEncoderValues(int & val_1, int & val_2);
  std::pair<bool, std::string> readBladeEncoderValue(int & val);
  std::pair<bool, std::string> setMotorValues(int val_1, int val_2);
  std::pair<bool, std::string> setBladeMotorValue(int val);
  std::pair<bool, std::string> setPidValues(float k_p, float k_d, float k_i, float k_o);
  void disconnect();

  bool connected() const { return serial_conn_.IsOpen(); }

  std::string sendMsg(const std::vector<uint8_t> & msg_to_send, bool print_output = false);

private:
  LibSerial::SerialPort serial_conn_;  ///< Underlying serial connection
};

#endif  // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H