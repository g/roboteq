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

#include <unistd.h>
#include <iostream>
#include <sstream>

#define ROBOTEQ_MAX_SETPOINT 1000 //This is defined by the roboteq software range is from -1000 to +1000

double last_path_error_callback=0;

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

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


void Interface::spinOnce() {
  // Process serial messages until there are no more characters waiting in the buffer.
  while(readSerial());
}


bool Interface::readSerial() {
  std::string response = serial_->readline(max_line_length, eol);

  if (response[0] == '-') {
    ROS_WARN_STREAM("CONTROL: Query error: " << response);
    return true;
  } else if (response[0] == '+') {
    ROS_DEBUG_STREAM("CONTROL: Query acked: " << response);
    return true;
  } else {
    // Call user callback with data.
    return callbacks_->handle(response);
  }
}



int Interface::sendSerialBlocking(std::string strQuery,std::string response) {

  char byteRead = 0;
  int time_out;
  int send_retries=0;
  int read_retries=0;
  response = "";

  while (response[0] != '+') {
    if (!serial_->write(strQuery)) {
      throw BadTransmission();
    }/*j else {

      if (response[0] == '-') {
        ROS_INFO("CONTROL: Query error-response:>%s", response.c_str());
        //return true;
      } else if (response[0] == '+') {
        ROS_DEBUG("CONTROL: Query ACKED on port %s send>  response:>%s",port_, response.c_str());
        return true;
      } else if (send_retries>3) {
        ROS_ERROR("CONTROL: Response Error With Serial Port %s Sent %s on try #%d response %s Giving Up",port_,strQuery.c_str(),read_retries,response.c_str());
        return (-2);
      }
      send_retries++;
    }*/
  }
  return false;
}

void Interface::sendSerial(std::string strQuery) {
  if (!serial_->write(strQuery)) {
    throw BadTransmission();
  }
}


void Interface::setMotorSpeeds() {
  std::string motor_command="";
  /*stringstream m1_string;
  stringstream m2_string;

  m1_string << motor_speed_[0];
  m2_string << motor_speed_[0];
  motor_command = "!M " + m1_string.str() + " " + m2_string.str() + "\r";*/
  sendSerial(motor_command);
}


void Interface::resetDIOx(int i) {
  std::string strCommand;
  std::stringstream stringID;
  stringID << i;
  strCommand="!D0 " + stringID.str() +"\r";
  sendSerial(strCommand);
}  // resetDIO

void Interface::setDIOx(int i) {
  // need to redo this one
  std::string strCommand;
  std::stringstream stringID;
  stringID << i;
  strCommand="!D1 " + stringID.str() +"\r";
  sendSerial(strCommand);
}  // setDIO

void Interface::setSetpoint(int motor, int val) {
  // need to redo this one
  std::string strCommand;
  std::stringstream stringMotor;
  std::stringstream stringVal;
  stringMotor << motor;
  //Bound the input
  if(val>ROBOTEQ_MAX_SETPOINT) {
    val=ROBOTEQ_MAX_SETPOINT;
  } else if(val<-ROBOTEQ_MAX_SETPOINT) {
    val=-ROBOTEQ_MAX_SETPOINT;
  }


  stringVal << val;
  strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
  last_command_sent_=strCommand;
  sendSerial(strCommand);
}  // setSetpoint

void Interface::setSetpoint(int val) {
  // need to redo this one
  std::string strCommand;
  std::stringstream stringVal;
  //Bound the input
  if(val>ROBOTEQ_MAX_SETPOINT) {
    val=ROBOTEQ_MAX_SETPOINT;
  } else if(val<-ROBOTEQ_MAX_SETPOINT) {
    val=-ROBOTEQ_MAX_SETPOINT;
  }
  stringVal << val;
  strCommand="!G " + stringVal.str() +"\r";
  sendSerial(strCommand);
}  // setSetpoint

void Interface::setEstop() {
  // need to redo this one
  std::string strCommand;
  strCommand="!EX \r";
  sendSerial(strCommand);
}  // setEstop

void Interface::resetEstop() {
  // need to redo this one
  std::string strCommand;
  strCommand="!MG \r";
  sendSerial(strCommand);
}

/*void Interface::setVAR(int i, int val)
{
    // need to redo this one
	string strCommand;
	stringstream stringID;
	stringstream stringVal;
	stringID << i;
	stringVal << val;
	strCommand="!VAR " + stringID.str() + " " + stringVal.str() +"\r";
	sendSerial(strCommand);
}  // setVAR


void Interface::readVAR(int i)
{
    //need to do this one
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	sendSerial(strQuery);
}  // readVAR
*/

int Interface::setMotorAmpLimit(uint8_t channel,float amp_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<amp_limit*10;
  command="^ALIM "+ channel_sel.str() + " " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Amp limit %d to %f ",channel,amp_limit);
    return false;
  }
  ROS_INFO("Set the amp limit on motor %d to %f",channel,amp_limit*10);
  return true;
}


int Interface::setMotorAmpLimit(float amp_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<amp_limit*10;
  command="^ALIM " + value.str() + "\r";
  if(sendSerialBlocking(command,response)) {
    return true;
  }
  ROS_WARN("Failed to set the motor amp limit to %f ",amp_limit);
  return false;
}

int Interface::setPIDICap(uint8_t channel,float pid_cap) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<pid_cap*100;
  command="^ICAP "+ channel_sel.str()+ " " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the PID Cap limit %d to %f ",channel,pid_cap);
    return false;

  }
  /*value<<pid_cap_2*100;
  command="^ICAP 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response))
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
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Ki for channel 1 to %f ",ki_1);
    return false;

  }
  value<<ki_2*10;
  command="^KI 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Ki for channel 2 to %f ",ki_2);
    return false;

  }

  return true;
}

int Interface::setPIDKp(uint8_t channel,float kp) {
  std::string response="";
  std::string command;
  std::stringstream value;
  std::stringstream channel_sel;
  channel_sel<<channel;
  value<<kp*10;
  command="^KP "+ channel_sel.str() + " " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Kp for channel %d to %f ",channel,kp);
    return false;

  }
  return true;
}

int Interface::setPIDkd(float kd_1,float kd_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<kd_1*10;
  command="^KD 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Kd for channel 1 to %f ",kd_1);
    return false;

  }
  value<<kd_2*10;
  command="^KD 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Kd for channel 2 to %f ",kd_2);
    return false;

  }

  return true;
}

int Interface::setMaxAcc(float max_acc_1,float max_acc_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_acc_1*10;
  command="^MAC 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max acceleration for channel 1 to %f ",max_acc_1);
    return false;

  }
  value<<max_acc_2*10;
  command="^MAC 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max acceleration for channel 2 to %f ",max_acc_2);
    return false;

  }

  return true;
}

int Interface::setMaxDec(float max_dcc_1,float max_dcc_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_dcc_1*10;
  command="^MDEC 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max deceleration for channel 1 to %f ",max_dcc_1);
    return false;

  }
  value<<max_dcc_2*10;
  command="^MDEC 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max deceleration for channel 2 to %f ",max_dcc_2);
    return false;

  }

  return true;
}

int Interface::setOperatingMode(uint8_t mode_1, uint8_t mode_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<mode_1*10;
  command="^MMOD 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the mode to %d ",mode_1);
    return false;

  }
  value<<mode_2*10;
  command="^MMOD 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the mode to %d ",mode_2);
    return false;

  }

  return true;
}


//this is not needed....
int Interface::setMaxPwrFwd(float max_fwd_1,float max_fwd_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_fwd_1*100;
  command="^MXPF 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max power forward to %f ",max_fwd_1);
    return false;

  }
  value<<max_fwd_2*100;
  command="^MXPF 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max power forward to %f ",max_fwd_2);
    return false;

  }

  return true;
}

int Interface::setMaxPwrRev(float max_rev_1,float max_rev_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_rev_1*100;
  command="^MXPR 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max power reverse to %f ",max_rev_1);
    return false;

  }
  value<<max_rev_2*100;
  command="^MXPR 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the max power reverse to %f ",max_rev_2);
    return false;

  }

  return true;
}


int Interface::setMaxRPM(uint16_t max_rpm_1,uint16_t max_rpm_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<max_rpm_1;
  command="^MRPM 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Max RPM to %d ",max_rpm_1);
    return false;

  }
  value<<max_rpm_2;
  command="^MRPM 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Max RPM to %d ",max_rpm_2);
    return false;

  }

  return true;
}

int Interface::setOvervoltageLimit(float overvoltage_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<overvoltage_limit*10;
  command="^OVL " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Overvoltage limit to %f ",overvoltage_limit);
    return false;
  }
  return true;
}

int Interface::setUndervoltageLimit(float undervoltage_limit) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<undervoltage_limit*10;
  command="^UVL " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Undervoltage limit to %f ",undervoltage_limit);
    return false;
  }
  return true;
}

int Interface::setEncoderPulsePerRev(uint16_t encoder_pulse_per_rev_1,uint16_t encoder_pulse_per_rev_2) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<encoder_pulse_per_rev_1;
  command="^EPPR 1 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Encoder pullse per revolutin PPR Channel 1 Value to %d ",encoder_pulse_per_rev_1);
    return false;
  }
  value<<encoder_pulse_per_rev_2;
  command="^EPPR 2 " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the Encoder pullse per revolutin PPR Channel 2 Value to %d ",encoder_pulse_per_rev_2);
    return false;
  }
  return true;
}


int Interface::setSerialEcho(bool serial_echo) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<serial_echo;
  command="^ECHOF " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    if(serial_echo)
      ROS_WARN("Failed to set the Serial Echo to 1 ");
    else
      ROS_WARN("Failed to set the Serial Echo to 0 ");

    return false;
  }
  return true;
}

int Interface::setSerialWatchdogTimeout(float wd_timeout) {
  std::string response="";
  std::string command;
  std::stringstream value;
  value<<wd_timeout*1000;
  command="^RWD " + value.str() + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the serial timeout to %f seconds", wd_timeout);
    return false;
  }
  return true;
}

int Interface::setAutomaticTelemetry(std::string telem_request) {
  std::string response="";
  std::string command;
  command="^TELS " + telem_request + "\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to set the telemetry string to %s", telem_request.c_str());
    return false;
  }
  return true;
}

int Interface::writeValuesToEEPROM() {
  std::string response="";
  std::string command;
  command="%EESAV\r";
  if(!sendSerialBlocking(command,response)) {
    ROS_WARN("Failed to save settings in RRPROM");
    return false;
  }
  return true;
}


}  // namespace roboteq
