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
  : callbacks_(callbacks), port_(port), baud_(baud), connected_(false), version_(""),
    command("!"), query("?"), param("^") {
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

/*void Interface::send(std::string msg) {
  std::string buf = msg + eol;
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << buf);
  serial_->write(buf);
}*/

/*void Interface::send(char type, std::string code, int8_t channel, std::string arg) {
  std::stringstream ss;
  ss << type << code;
  if (channel > 0) {
    ss << " " << channel;
  }
  if (!arg.empty()) {
    ss << " " << arg;
  }
  ss << eol;
  std::string buf(ss.str());
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << buf);
  serial_->write(buf);
}*/

/*void Interface::send(char type, std::string code, std::string arg) {
  send(type, code, 0, arg);
}*/

/*std::string Interface::sendWaitReply(std::string msg, int tries) {
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

}*/

}  // namespace roboteq
