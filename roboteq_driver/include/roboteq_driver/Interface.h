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

#include <boost/thread/condition_variable.hpp>
#include <stdint.h>
#include <string>

namespace serial {
  class Serial;
}

namespace roboteq {

class Callbacks;

class Interface {
private :
  Callbacks* callbacks_;
  const char *port_;
  int baud_;
  bool connected_;
  std::string version_;
  serial::Serial *serial_;

  void send(std::string msg);
  std::string sendWaitReply(std::string msg, int tries=5);
  bool sendWaitAck(std::string msg);

  void read();

  // These data members are the core of the synchronization strategy in this class.
  // In short, the sendWaitAck method blocks on receiving an ack, which is passed to
  // it from the read thread using the last_response_ string.
  std::string last_response_;
  boost::mutex last_response_mutex_;
  boost::condition_variable last_response_available_;
  bool haveLastResponse() { return !last_response_.empty(); }

public :
  Interface (const char *port, int baud, Callbacks* callbacks);
  ~Interface();
  void connect();
  bool connected() { return connected_; }
  void spinOnce() { read(); }

  // Send commands to motor driver.
  void setMotorSpeeds();
  void resetDIOx(int i);
  void setDIOx(int i);
  void setSetpoint(int motor, int val);
  void setEstop();
  void resetEstop();
  void setVAR(int i, int val);

  // Runtime queries: the results of these go to the callback methods.
  void getMotorCurrent() { send("?A"); }
  void getSupplyCurrent() { send("?BA"); }
  void getEncoderCount() { send("?C"); }
  void getEncoderRPM() { send("?S"); }
  void getDigitalInputs();
  void getDigitalOutputs();
  void getClosedLoopError() { send("?E"); }
  void getFeedbackIn();
  void getFault() { send("?FF"); }
  void getStatus() { send("?FS"); }
  void getMotorPower() { send("?P"); }
  void getPulsedInputs();
  void getDriverTemperature() { send("?T"); }
  void getMotorTemperature() { send("?AI 1"); }
  void getMotorCommanded() { send("?M"); }
  void getVoltages() { send("?V"); }

  // Set configuration parameters, when writing these values should be confirmed (aka should probably block)
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
  int setAutomaticTelemetry(std::string telem_request);
  int writeValuesToEEPROM();
};

}

#endif
