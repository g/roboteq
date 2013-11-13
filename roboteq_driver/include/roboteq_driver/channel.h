/**
Software License Agreement (BSD)

\file      channel.h
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
#ifndef ROBOTEQ_CHANNEL
#define ROBOTEQ_CHANNEL

#include "ros/ros.h"

namespace roboteq_msgs {
  ROS_DECLARE_MESSAGE(Command);
  ROS_DECLARE_MESSAGE(Feedback);
}

namespace roboteq {

class Controller;

class Channel {
public:
  Channel(int channel_num, std::string ns, Controller* controller);
  void feedbackCallback(std::vector<std::string>);

protected:
  void cmdCallback(const roboteq_msgs::Command&);
  void timerCallback(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  boost::shared_ptr<Controller> controller_;
  int channel_num_;
  float max_rpm_;

  ros::Subscriber sub_cmd_;
  ros::Publisher pub_feedback_;
  ros::Timer timer_init_;

  ros::Time last_feedback_time_;
};

}

#endif
