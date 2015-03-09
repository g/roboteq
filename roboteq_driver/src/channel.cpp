/**
Software License Agreement (BSD)

\file      channel.cpp
\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
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

#include "roboteq_driver/channel.h"
#include "roboteq_driver/controller.h"

#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"
#include "roboteq_msgs/Command.h"

// Simple conversions between rad/s and RPM.
#define TO_RPM(x) (double(x) * 60 / (2 * M_PI))
#define FROM_RPM(x) (double(x) * (2 * M_PI) / 60)

namespace roboteq {

Channel::Channel(int channel_num, std::string ns, Controller* controller) :
  channel_num_(channel_num), nh_(ns), controller_(controller), max_rpm_(3500)
{
  sub_cmd_ = nh_.subscribe("cmd", 1, &Channel::cmdCallback, this);
  pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);

  timer_init_ = nh_.createTimer(ros::Duration(1.0), &Channel::timerCallback, this);
}

void Channel::cmdCallback(const roboteq_msgs::Command& command) {
  // First convert the user's command from rad/s to RPM.
  float commanded_rpm = TO_RPM(command.commanded_velocity);

  // Now get the -1000 .. 1000 command as a proportion of the maximum RPM.
  int roboteq_command = int((commanded_rpm / max_rpm_) * 1000.0);
  ROS_DEBUG_STREAM("Sending command value of " << roboteq_command << " to motor driver.");

  // Write the command.
  controller_->command << "G" << channel_num_ << roboteq_command << controller_->send;
  controller_->flush();
}

void Channel::feedbackCallback(std::vector<std::string> fields) {
  roboteq_msgs::Feedback msg;
  msg.header.stamp = last_feedback_time_ = ros::Time::now();

  try {
    msg.motor_current = boost::lexical_cast<float>(fields[2]) / 10;
    msg.commanded_velocity = FROM_RPM(boost::lexical_cast<double>(fields[3]));
    msg.motor_power = boost::lexical_cast<float>(fields[4]) / 1000.0;
    msg.measured_velocity = FROM_RPM(boost::lexical_cast<double>(fields[5]));
    msg.measured_position = boost::lexical_cast<double>(fields[6]) * 2 * M_PI / 4096;
    msg.supply_voltage = boost::lexical_cast<float>(fields[7]) / 10.0;
    msg.supply_current = boost::lexical_cast<float>(fields[8]) / 10.0;
    msg.motor_temperature = boost::lexical_cast<int>(fields[9]) * 0.020153 - 4.1754;
    msg.channel_temperature = boost::lexical_cast<int>(fields[10]);
  } catch (std::bad_cast& e) {
    ROS_WARN("Failure parsing feedback data. Dropping message.");
    return;
  }
  pub_feedback_.publish(msg);
}

void Channel::timerCallback(const ros::TimerEvent&) {
  if (ros::Time::now() - last_feedback_time_ > ros::Duration(1.0)) {
    // Not receiving feedback, attempt to start it.
    controller_->setUserBool(channel_num_, 1);
    controller_->flush();
  }
}

}
