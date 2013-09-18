/**
Software License Agreement (BSD)

\file      Interface.cpp
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
#include "roboteq_driver/Interface.h"

#include "roboteq_driver/Callbacks.h"

#include "serial/serial.h"
#include "ros/ros.h"

#include <boost/bind.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

// Roboteq-defined limit
const int max_setpoint(1000);

Interface::Interface (const char *port, int baud, Callbacks* callbacks)
  : callbacks_(callbacks), port_(port), baud_(baud), connected_(false), version_("") {
    class Callbacks;
}

Interface::~Interface() {
}

void Interface::connect() {
  serial_ = new serial::Serial(port_, baud_);

  for (int tries = 0; tries < 5; tries++) {
    std::string cmd("?FID" + eol);
    if (serial_->write(cmd) == cmd.length()) {
      if (serial_->isOpen())
        connected_ = true;
      else {
        connected_ = false;
        ROS_INFO("Bad Connection with serial port Error %s",port_);
        throw BadConnection();
      }
      std::string command_readback = serial_->readline(max_line_length, eol);
      std::string version_readback = serial_->readline(max_line_length, eol);
      ROS_INFO_STREAM("Motor controller on port " << port_ << " version [" << version_readback << "]");
      return;
    } else {
      throw BadConnection();
    }
    tries++;
  }

  ROS_INFO("Motor controller not responding.");
  throw BadConnection();
}

void Interface::read() {
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
  if (msg[0] == '+' || msg[0] == '-') {
    // Notify the ROS thread that a message response (ack/nack) has arrived.
    boost::lock_guard<boost::mutex> lock(last_response_mutex_);
    last_response_ = msg;
    last_response_available_.notify_one();
  } else {
    // User callback to handle it.
    callbacks_->handle(msg);
  }
}

void Interface::send(std::string msg) {
  std::string buf = msg + eol;
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << buf);
  serial_->write(buf);
}

std::string Interface::sendWaitReply(std::string msg, int tries) {
  for (int trynum = 0; trynum <= tries; trynum++ ) {
    boost::unique_lock<boost::mutex> lock(last_response_mutex_);
    boost::system_time const timeout(boost::get_system_time() + boost::posix_time::milliseconds(500));
    last_response_.clear();

    send(msg);
    if (last_response_available_.timed_wait(lock, timeout,
          boost::bind(&Interface::haveLastResponse, this))) {
      // Response available.
      ROS_DEBUG_STREAM_NAMED("comms", "Reply to message received.");
      return last_response_;
    } else {
      // Timed out waiting for response, try again.
      ROS_WARN_STREAM_NAMED("comms", "Missing reply to message, retry #" << trynum);
      continue;
    }
  }
  ROS_ERROR_STREAM_NAMED("comms", "Giving up on receiving reply to message [" << msg << "]");
  throw BadTransmission();
}

bool Interface::sendWaitAck(std::string msg) {

}

void Interface::setMotorSpeeds() {
  std::string motor_command="";
  /*stringstream m1_string;
  stringstream m2_string;

  m1_string << motor_speed_[0];
  m2_string << motor_speed_[0];
  motor_command = "!M " + m1_string.str() + " " + m2_string.str() + "\r";*/
  //sendSerial(motor_command);
}

void Interface::resetDIOx(int i) {
  std::string strCommand;
  std::stringstream stringID;
  stringID << i;
  strCommand="!D0 " + stringID.str() +"\r";
  send(strCommand);
}  // resetDIO

void Interface::setDIOx(int i) {
  // need to redo this one
  std::string strCommand;
  std::stringstream stringID;
  stringID << i;
  strCommand="!D1 " + stringID.str() +"\r";
  send(strCommand);
}  // setDIO

void Interface::setSetpoint(int motor, int val) {
  // need to redo this one
  std::string strCommand;
  std::stringstream stringMotor;
  std::stringstream stringVal;
  stringMotor << motor;
  //Bound the input
  if(val > max_setpoint) {
    val = max_setpoint;
  } else if(val < -max_setpoint) {
    val = -max_setpoint;
  }
  stringVal << val;
  strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
  send(strCommand);
}  // setSetpoint

void Interface::setEstop() {
  // need to redo this one
  std::string strCommand;
  strCommand="!EX \r";
  send(strCommand);
}  // setEstop

void Interface::resetEstop() {
  // need to redo this one
  std::string strCommand;
  strCommand="!MG \r";
  send(strCommand);
}

int Interface::setMotorAmpLimit(uint8_t channel,float amp_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<amp_limit*10;
  command="^ALIM "+ channel_sel.str() + " " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setMotorAmpLimit(float amp_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<amp_limit*10;
  command="^ALIM " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setPIDICap(uint8_t channel,float pid_cap) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<pid_cap*100;
  command="^ICAP "+ channel_sel.str()+ " " + value.str() + "\r";
  sendWaitAck(command);
  /*value<<pid_cap_2*100;
  command="^ICAP 2 " + value.str() + "\r";
  if(!sendBlocking(command,response))
  {
  	ROS_WARN("Failed to set the PID Cap limit 2 to %f ",pid_cap_2);
  	return false;

  }*/

  return true;
}

int Interface::setPIDKi(float ki_1,float ki_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<ki_1*10;
  command="^KI 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<ki_2*10;
  command="^KI 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setPIDKp(uint8_t channel,float kp) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<kp*10;
  command="^KP "+ channel_sel.str() + " " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setPIDkd(float kd_1,float kd_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<kd_1*10;
  command="^KD 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<kd_2*10;
  command="^KD 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setMaxAcc(float max_acc_1,float max_acc_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_acc_1*10;
  command="^MAC 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<max_acc_2*10;
  command="^MAC 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setMaxDec(float max_dcc_1,float max_dcc_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_dcc_1*10;
  command="^MDEC 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<max_dcc_2*10;
  command="^MDEC 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setOperatingMode(uint8_t mode_1, uint8_t mode_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<mode_1*10;
  command="^MMOD 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<mode_2*10;
  command="^MMOD 2 " + value.str() + "\r";
  sendWaitAck(command);
}


//this is not needed....
int Interface::setMaxPwrFwd(float max_fwd_1,float max_fwd_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_fwd_1*100;
  command="^MXPF 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<max_fwd_2*100;
  command="^MXPF 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setMaxPwrRev(float max_rev_1,float max_rev_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_rev_1*100;
  command="^MXPR 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<max_rev_2*100;
  command="^MXPR 2 " + value.str() + "\r";
  sendWaitAck(command);
}


int Interface::setMaxRPM(uint16_t max_rpm_1,uint16_t max_rpm_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_rpm_1;
  command="^MRPM 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<max_rpm_2;
  command="^MRPM 2 " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setOvervoltageLimit(float overvoltage_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<overvoltage_limit*10;
  command="^OVL " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setUndervoltageLimit(float undervoltage_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<undervoltage_limit*10;
  command="^UVL " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setEncoderPulsePerRev(uint16_t encoder_pulse_per_rev_1,uint16_t encoder_pulse_per_rev_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<encoder_pulse_per_rev_1;
  command="^EPPR 1 " + value.str() + "\r";
  sendWaitAck(command);
  value<<encoder_pulse_per_rev_2;
  command="^EPPR 2 " + value.str() + "\r";
  sendWaitAck(command);
}


int Interface::setSerialEcho(bool serial_echo) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<serial_echo;
  command="^ECHOF " + value.str() + "\r";
}

int Interface::setSerialWatchdogTimeout(float wd_timeout) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<wd_timeout*1000;
  command="^RWD " + value.str() + "\r";
  sendWaitAck(command);
}

int Interface::setAutomaticTelemetry(std::string telem_request) {
  std::string response="";
  std::string command;
  command="^TELS " + telem_request + "\r";
  sendWaitAck(command);
}

int Interface::writeValuesToEEPROM() {
  std::string response="";
  std::string command;
  command="%EESAV\r";
  sendWaitAck(command);
}


}  // namespace roboteq
