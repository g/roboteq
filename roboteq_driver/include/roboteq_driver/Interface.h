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

#include <boost/thread/condition_variable.hpp>
#include <boost/lexical_cast.hpp>
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
  std::stringstream tx_buffer_;

  // These data members are the core of the synchronization strategy in this class.
  // In short, the sendWaitAck method blocks on receiving an ack, which is passed to
  // it from the read thread using the last_response_ string.
  std::string last_response_;
  boost::mutex last_response_mutex_;
  boost::condition_variable last_response_available_;
  bool haveLastResponse() { return !last_response_.empty(); }

  // These track our progress in attempting to initialize the controller.
  uint8_t start_script_attempts_;
  bool received_feedback_;

  class EOMSend {};

  class MessageSender {
    public:
    MessageSender(std::string init, Interface* interface)
        : init_(init), interface_(interface) {}

    template<typename T>
    MessageSender& operator<<(const T val) {
      if (ss.tellp() == 0) {
        ss << init_ << val;
      } else {
        ss << ' ' << val;
      }
      return *this;
    }
 
    void operator<<(EOMSend) 
    {
      interface_->write(ss.str());
      ss.str("");
    }
   
    private:
    std::string init_;
    Interface* interface_;
    std::stringstream ss;
  };

  MessageSender command;
  MessageSender query;
  MessageSender param;
  EOMSend send, sendVerify;

  void read();
  void write(std::string);
  
  void processStatus(std::string msg);
  void processFeedback(std::string msg);
 
public :
  Interface (const char *port, int baud);
  ~Interface();
  void connect();
  bool connected() { return connected_; }
  void spinOnce() { read(); }
  void flush();

  // Send commands to motor driver.
  void setEstop() { command << "EX" << send; }
  void resetEstop() { command << "MG" << send; }
  void resetDIOx(int i) { command << "D0" << i << send; }
  void setDIOx(int i) { command << "D1" << i << send; }
  void setSetpoint(int channel, int val) { command << "G" << channel << val << send; }
  void startScript() { command << "R" << send; }
  void stopScript() { command << "R" << 0 << send; }
  void setUserVariable(int var, int val) { command << "VAR" << var << val << send; }
  void setUserBool(int var, bool val) { command << "B" << var << (val ? 1 : 0) << send; }
  bool downloadScript();

  // Runtime queries: the results of these go to the callback methods.
  void getMotorCurrent() { query << "A" << send; }
  void getSupplyCurrent() { query << "BA" << send; }
  void getEncoderCount() { query << "C" << send; }
  void getEncoderRPM() { query << "S" << send; }
  void getClosedLoopError() { query << "E" << send; }
  void getFault() { query << "FF" << send; }
  void getStatus() { query << "FS" << send; }
  void getMotorPower() { query << "P" << send; }
  void getDriverTemperature() { query << "T" << send; }
  void getMotorTemperature() { query << "AI" << send; }
  void getMotorCommanded() { query << "M" << send; }
  void getVoltages() { query << "V" << send; }

  // Set configuration parameters.
  int setMotorAmpLimit(uint8_t channel,float amp_limit) { 
    param << "ALIM" << channel << amp_limit * 10 << sendVerify; }
  int setPIDICap(uint8_t channel, float i_cap) { 
    param << "ICAP" << channel << i_cap * 100 << sendVerify; }
  int setPIDKp(uint8_t channel,float kp) { 
    param << "KP" << channel << kp * 10 << sendVerify; }
  int setPIDkd(uint8_t channel, float kd) { 
    param << "KD" << channel << kd * 10 << sendVerify; }
  int setMaxAcc(uint8_t channel, float rpm_per_second) { 
    param << "MAC" << channel << rpm_per_second * 10 << sendVerify; }
  int setMaxDec(uint8_t channel, float rpm_per_second) { 
    param << "MDEC" << channel << rpm_per_second * 10 << sendVerify; }
  int setOperatingMode(uint8_t channel, uint8_t mode) {
    param << "MMOD" << channel << mode << sendVerify; }
  int setMaxPwrFwd(uint8_t channel, float max_fwd) { 
    param << "MXPF" << channel << max_fwd * 100 << sendVerify; }
  int setMaxPwrRev(uint8_t channel, float max_rev) { 
    param << "MXPR" << channel << max_rev * 100 << sendVerify; }
  int setMaxRPM(uint8_t channel, uint16_t max_rpm) { 
    param << "MXRPM" << channel << max_rpm << sendVerify; }
  int setOvervoltageLimit(float limit) { 
    param << "OVL" << limit * 10 << sendVerify; }
  int setUndervoltageLimit(float limit) { 
    param << "UVL" << limit * 10 << sendVerify; }
  int setEncoderPulsePerRev(uint8_t channel, uint16_t pulse_per_rev) {
    param << "EPPR" << channel << pulse_per_rev << sendVerify; }
  int setSerialEcho(bool serial_echo) {
    param << "ECHOF" << (serial_echo ? 0 : 1) << sendVerify; }
  int setSerialWatchdogTimeout(int timeout_ms) { 
    param << "RWD" << timeout_ms << sendVerify; }

  //int setAutomaticTelemetry(std::string telem_request);
  //int writeValuesToEEPROM();
};

}

#endif
