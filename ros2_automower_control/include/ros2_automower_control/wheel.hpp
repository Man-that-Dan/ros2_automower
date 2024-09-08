// taken from : https://github.com/joshnewans/diffdrive_arduino/blob/main/include/diffdrive_arduino/wheel.h
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

#ifndef ROS2_AUTOMOWER_CONTROL___WHEEL_H
#define ROS2_AUTOMOWER_CONTROL___WHEEL_H

#include <string>

class Wheel
{
public:
  std::string name = "";
  int enc = 0;
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double eff = 0;
  double velSetPt = 0;
  double rads_per_count = 0;

  Wheel() = default;

  Wheel(const std::string & wheel_name, int counts_per_rev);

  void setup(const std::string & wheel_name, int counts_per_rev);

  double calcEncAngle();
};

#endif  // ROS2_AUTOMOWER_CONTROL___WHEEL_H