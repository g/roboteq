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

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <unistd.h>
#include <iostream>
#include <sstream>

// Link to generated source from Microbasic script file. 
extern const char* script_lines[];

namespace roboteq {

const std::string eol("\r");
const size_t max_line_length(128);

// Roboteq-defined limit
const int max_setpoint(1000);

Interface::Interface (const char *port, int baud)
  : port_(port), baud_(baud), connected_(false), version_(""),
    start_script_attempts_(0), received_feedback_(false),
    command("!", this), query("?", this), param("^", this) {
}

Interface::~Interface() {
}

void Interface::connect() {
  serial_ = new serial::Serial(port_, baud_);

  serial::Timeout to = serial::Timeout::simpleTimeout(500);
  serial_->setTimeout(to);

  for (int tries = 0; tries < 5; tries++) {
    query << "FID" << send;
    setSerialEcho(false);
    flush();

    if (serial_->isOpen())
      connected_ = true;
    else {
      connected_ = false;
      ROS_INFO("Bad Connection with serial port Error %s",port_);
      throw BadConnection();
    }

    return;
  }

  ROS_INFO("Motor controller not responding.");
  throw BadConnection();
}

void Interface::read() {
  std::string msg = serial_->readline(max_line_length, eol);
  if (!msg.empty()) {
    ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
    if (msg[0] == '+' || msg[0] == '-') {
      // Notify the ROS thread that a message response (ack/nack) has arrived.
      boost::lock_guard<boost::mutex> lock(last_response_mutex_);
      last_response_ = msg;
      last_response_available_.notify_one();
    } else if (msg[0] == '&') {
      if (msg[1] == 's') {
        // Motor controller status. 
        processStatus(msg);
        if (!received_feedback_) {
          ROS_DEBUG("Attempt to start feedback output.");
          setUserBool(1, 1);
          flush();
        }
        received_feedback_ = false;
      } else if (msg[1] == 'f') {
        // Motor controller channel feedback.
        processFeedback(msg);
        received_feedback_ = true;
      } 
    } else {
      // Unknown other message.
      ROS_WARN_STREAM("Unknown serial message received: " << msg);
    }
  } else {
    ROS_WARN_NAMED("serial", "Serial::readline() returned no data.");
    ROS_DEBUG("No status messages.");
    if (start_script_attempts_ < 5) {
      start_script_attempts_++;
      ROS_DEBUG("Attempt #%d to start MBS program.", start_script_attempts_);
      startScript();
      flush();
      //ros::Duration(0.2).sleep(); 
    } else {
      ROS_DEBUG("Attempting to download MBS program.");
      if (downloadScript()) {
        start_script_attempts_ = 0;
      }
      ros::Duration(1.0).sleep(); 
    }
  }
}

void Interface::write(std::string msg) {
  tx_buffer_ << msg << eol;
}

void Interface::flush() {
  ROS_DEBUG_STREAM_NAMED("serial", "TX: " << boost::algorithm::replace_all_copy(tx_buffer_.str(), "\r", "\\r"));
  ssize_t bytes_written = serial_->write(tx_buffer_.str());
  if (bytes_written < tx_buffer_.tellp()) {
    ROS_WARN_STREAM("Serial write timeout, " << bytes_written << " bytes written of " << tx_buffer_.tellp() << ".");
  }
  tx_buffer_.str("");
}

void Interface::processStatus(std::string msg) {

}

void Interface::processFeedback(std::string msg) {

}

bool Interface::downloadScript() {
  ROS_DEBUG("Commanding driver to stop executing script.");
  stopScript(); flush();
  serial_->readline(max_line_length, eol);  // Swallow ack/nack.
  
  // Send SLD.
  ROS_DEBUG("Commanding driver to enter download mode.");
  write("%SLD 321654987"); flush();

  // Check special ack from SLD.
  std::string msg = serial_->readline(max_line_length, eol);
  ROS_DEBUG_STREAM_NAMED("serial", "RX: " << msg);
  if (msg != "HLD\r") {
    ROS_DEBUG("Could not enter download mode.");
    return false;
  }
  
  // Send hex program, line by line, checking for an ack from each line.
  int line_num = 0;
  while(script_lines[line_num]) {
    std::string line(script_lines[line_num]);
    write(line);
    flush();
    std::string ack = serial_->readline(max_line_length, eol);
    ROS_DEBUG_STREAM_NAMED("serial", "RX: " << ack);
    if (ack != "+\r") return false;
    line_num++;
  }
  return true;
}

}  // namespace roboteq
