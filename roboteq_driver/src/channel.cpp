
#include "roboteq_driver/channel.h"
#include "roboteq_driver/controller.h"

#include "ros/ros.h"
#include "roboteq_msgs/Feedback.h"
#include "roboteq_msgs/Command.h"


namespace roboteq {

Channel::Channel(uint8_t channel_num, std::string ns, Controller* controller) :
  channel_num_(channel_num), nh_(ns), controller_(controller)
{
  sub_cmd_ = nh_.subscribe("cmd", 1, &Channel::cmdCallback, this);
  pub_feedback_ = nh_.advertise<roboteq_msgs::Feedback>("feedback", 1);

  timer_init_ = nh_.createTimer(ros::Duration(1.0), &Channel::timerCallback, this);
}

void Channel::cmdCallback(const roboteq_msgs::Command& command) {
  // TODO: max rpm
  controller_->command << "G" << channel_num_ << command.commanded_velocity << controller_->send;
  controller_->flush();
}

void Channel::feedbackCallback(std::vector<std::string> fields) {
  roboteq_msgs::Feedback msg;
  msg.header.stamp = last_feedback_time_ = ros::Time::now();

  try {
    msg.motor_current = boost::lexical_cast<int>(fields[2]);
    msg.commanded_velocity = boost::lexical_cast<int>(fields[3]);
    msg.motor_power = boost::lexical_cast<int>(fields[4]);
    msg.measured_velocity = boost::lexical_cast<int>(fields[5]);
    msg.measured_position = boost::lexical_cast<int>(fields[6]);
    msg.supply_voltage = boost::lexical_cast<int>(fields[7]);
    msg.supply_current = boost::lexical_cast<int>(fields[8]);
    msg.motor_temperature = boost::lexical_cast<int>(fields[9]);
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
    controller_->setUserBool(1, 1);
    controller_->flush();
  }
}

}
