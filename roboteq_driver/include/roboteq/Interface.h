/**
Software License Agreement (BSD)

\file      Interface.h
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
#ifndef ROBOTEQ_INTERFACE
#define ROBOTEQ_INTERFACE

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboteq/lightweightserial.h"
#include "roboteq/Callbacks.h"
#include <unistd.h>
#include <iostream>
#include <sstream>

#define TRUE 1
#define FALSE 0

#define ASCII_CR_CODE 13
#define ASCII_ACK_CODE 43

#define ROBOTEQ_MAX_SETPOINT 1000 //This is defined by the roboteq software range is from -1000 to +1000

using namespace std;

namespace roboteq {

class Interface {
private :
  Callbacks* callbacks_;
  const char *port_;
  int baud_;
  bool connected_;
  std::string version_;
  LightweightSerial *controllerPort;

  void sendSerial(string strQuery);
  int sendSerialBlocking(string strQuery, string response);

  bool readSerial();
  std::string last_command_sent_;

public :
  Interface (const char *port, int baud, Callbacks* callbacks);
  ~Interface();
  void connect();
  bool connected() {
    return connected_;
  }
  void spinOnce();


  void setMotorSpeeds();
  void resetDIOx(int i);
  void setDIOx(int i);
  void setSetpoint(int motor,int val);
  void setSetpoint(int val);
  void setEstop();
  void resetEstop();
  void setVAR(int i, int val);

  // Runtime Querys.
  void getMotorCurrent() {
    sendSerial("?A\r");
  }
  void getSupplyCurrent() {
    sendSerial("?BA\r");
  }
  void getEncoderCount() {
    sendSerial("?C\r");
  }
  void getEncoderRPM() {
    sendSerial("?S\r");
  }
  void getDigitalInputs();
  void getDigitalOutputs();
  void getClosedLoopError() {
    sendSerial("?E\r");
  }
  void getFeedbackIn();
  void getFault() {
    sendSerial("?FF\r");
  }
  void getStatus() {
    sendSerial("?FS\r");
  }
  void getMotorPower() {
    sendSerial("?P\r");
  }
  void getPulsedInputs();
  void getDriverTemperature() {
    sendSerial("?T\r");
  }
  void getMotorTemperature() {
    sendSerial("?AI 1\r");
  }
  void getMotorCommanded() {
    sendSerial("?M\r");
  }
  void getUserVariable() {
    sendSerial("?VAR\r");
  }
  void getVoltages() {
    sendSerial("?V\r");
  }

  //Configuration Parameters, when writing these values should be confirmed (aka should probably block)

  int setMotorAmpLimit(float amp_limit);
  int setMotorAmpLimit(uint8_t channel,float amp_limit);
  int setPIDICap(uint8_t channel, float pid_cap);
  int setPIDKi(float ki_1,float ki_2);
  int setPIDKp(uint8_t channel,float kp);
  int setPIDkd(float kd_1,float kd_2);
  int setMaxAcc(float max_acc_1,float max_acc_2);
  int setMaxAcc(float max_acc);
  int setMaxDec(float max_dcc);
  int setMaxDec(float max_dcc_1,float max_dec_2);
  int setOperatingMode(uint8_t mode_1,uint8_t mode_2);
  int setMaxPwrFwd(float max_fwd_1,float max_fwd_2);
  int setMaxPwrFwd(float max_fwd);
  int setMaxPwrRev(float max_rev_1,float max_rev_2);
  int setMaxPwrRev(float max_rev_1);
  int setMaxRPM(uint16_t max_rpm_1,uint16_t max_rpm_2);
  int setMaxRPM(uint16_t max_rpm);
  int setOvervoltageLimit(float overvoltage_limit);
  int setUndervoltageLimit(float undervoltage_limit);
  int setEncoderPulsePerRev(uint16_t encoder_pulse_per_rev_1,uint16_t encoder_pulse_per_rev_2);
  int setSerialEcho(bool serial_echo);
  int setSerialWatchdogTimeout(float wd_timeout);
  int setAutomaticTelemetry(string telem_request);
  int writeValuesToEEPROM();



};

}

#endif
