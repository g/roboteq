/**
Software License Agreement (BSD)

\file      Callbacks.h
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
#ifndef ROBOTEQ_CALLBACKS
#define ROBOTEQ_CALLBACKS

#include "roboteq_driver/exceptions.h"

#include <stdint.h>
#include <string>

namespace roboteq {

class Callbacks {
private:
  virtual void motorCurrent(float current_1, float current_2) {
    throw NoHandler();
  }
  virtual void supplyCurrent(float battery) {
    throw NoHandler();
  }
  virtual void closedLoopError(uint8_t error) {
    throw NoHandler();
  }
  virtual void motorCommanded(float commanded_1, float commanded_2) {
    throw NoHandler();
  }
  virtual void encoderCount(uint32_t ticks_1, uint32_t ticks_2) {
    throw NoHandler();
  }
  virtual void encoderRPM(uint32_t rpm_1, uint32_t rpm_2) {
    throw NoHandler();
  }
  virtual void motorPower(float power_1, float power_2) {
    throw NoHandler();
  }
  virtual void voltages(float drive, float battery, float analog) {
    throw NoHandler();
  }
  virtual void versionID(std::string ID) {
    throw NoHandler();
  }
  virtual void controllerStatus(uint8_t c_status) {
    throw NoHandler();
  }
  virtual void controllerFault(uint8_t fault) {
    throw NoHandler();
  }
  virtual void controllerTemperatue(uint8_t fault) {
    throw NoHandler();
  }
  virtual void motorTemperature(float m_temperature) {
    throw NoHandler();
  }
  virtual void driverTemperature(int temperature_ic,int temperature_chan1,int temperature_chan2) {
    throw NoHandler();
  }

  /*virtual void logdebug(string s);
  virtual void loginfo(string s);
  virtual void logwarn(string s);*/

  // Internal helper functions
  bool call(std::string code, std::string fields[]);
  static float to_float(std::string field, float scale=0.1);
  static int32_t to_int(std::string field);

public:
  bool handle(std::string);
};

}

#endif
