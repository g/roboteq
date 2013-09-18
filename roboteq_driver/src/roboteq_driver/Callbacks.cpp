/**
Software License Agreement (BSD)

\file      Callbacks.cpp
\authors   Mike Irvine <mirvine@clearpathrobotics.com>
\copyright Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "roboteq_driver/Callbacks.h"

#include "ros/ros.h"

#include <stdlib.h>

namespace roboteq {

bool Callbacks::handle(std::string response) {
  //  ROS_INFO("Response: %s", response.c_str());

  size_t equals_sign = response.find("=");
  if (equals_sign == std::string::npos) {
    // Line which includes no data.
    return false;
  }
  std::string code = response.substr(0, equals_sign);

  // Split up response into multiple fields.
  response = response.substr(equals_sign + 1);
  size_t delimiter = 0;
  uint8_t i = 0;
  std::string fields[20];
  while (delimiter != std::string::npos) {
    delimiter = response.find(":");
    std::string field = response.substr(0, delimiter).c_str();
    // ROS_INFO("  field[%d] = %s \n", i, field.c_str());
    fields[i++] = field;
    response = response.substr(delimiter + 1);
  }


  return call(code, fields);
}

float Callbacks::to_float(std::string field, float scale) {
  return atoi(field.c_str()) * scale;
}

int32_t Callbacks::to_int(std::string field) {
  return atoi(field.c_str());
}


bool Callbacks::call(std::string code, std::string fields[]) {

  // Decide what handler to call. Use a switch for single-character
  // codes, and a cascaded if for the others.
  if (code.length() == 1) {
    switch (code[0]) {
    case 'A':
      motorCurrent(to_float(fields[0]), to_float(fields[1]));
      break;
    case 'E':
      closedLoopError(to_int(fields[0]));
      break;
    case 'M':
      motorCommanded(to_float(fields[0],10), to_float(fields[1],10));
      break;
    case 'C':
      encoderCount(to_float(fields[0]), to_float(fields[1]));
      break;
    case 'S':
      encoderRPM(to_int(fields[0]), to_int(fields[1]));
      break;
    case 'P':
      motorPower(to_float(fields[0]), to_float(fields[1]));
      break;
    case 'V':
      voltages(to_float(fields[0]), to_float(fields[1]), to_float(fields[2], 0.001));
      break;
    case 'T':
      driverTemperature(to_int(fields[0]), to_int(fields[1]), to_int(fields[2]));
      break;
    default:
      //ROS_WARN("Unhandled code: %s", code);
      return false;
    }
  } else if (code.compare("FID") == 0) {
    versionID(fields[0]);
  } else if (code.compare("VAR") == 0) {
    //userVariable();
  } else if (code.compare("BA") == 0) {
    supplyCurrent(to_float(fields[0]));
  } else if (code.compare("FS") == 0) {
    controllerStatus(to_int(fields[0]));
  } else if (code.compare("FF") == 0) {
    controllerFault(to_int(fields[0]));
  } else if (code.compare("AI") == 0) {
    motorTemperature(to_float(fields[0])); //only getting one
  } else {

    // ROS_WARN("Unhandled code: %s", code)
    return false;
  }
  return true;
}

}
