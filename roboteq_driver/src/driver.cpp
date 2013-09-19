/**
Software License Agreement (BSD)

\file      driver.cpp
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
#include "roboteq_driver/exceptions.h"

#include "ros/ros.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Status.h"
#include "roboteq_msgs/Feedback.h"
#include "roboteq_msgs/Config.h"

roboteq::Interface* controller;


void command_callback(const roboteq_msgs::Command& command) {
  unsigned int i;
  for (i=0; i<command.setpoint.size(); i++) {
    //ROS_INFO("Sending %f to motor %d size2 %d", command.setpoint[i],i+1,command.setpoint.size());
    //set the value of the motors
    controller->setSetpoint((i+1), command.setpoint[i]);
  }
}

void config_callback(const roboteq_msgs::Config& config) {
  unsigned int i;
  /*for (i=0; i<config.motor_amp_limit.size(); i++) {
    controller->setMotorAmpLimit((i+1), config.motor_amp_limit[i]);
    ROS_INFO("Set the motor current limit of motor %d to %f",i+1,config.motor_amp_limit[i]);
  }*/
}

class Callbacks : public roboteq::Callbacks {
private:
  // Feedback message handlers
  uint8_t feedback_pending;

  roboteq_msgs::Feedback feedback;
  ros::Publisher feedback_publisher;

  void request_feedback() {
    controller->getMotorCurrent();
    controller->getMotorCommanded();
    controller->getMotorPower();
    controller->getEncoderRPM();
    controller->getVoltages();
    controller->getSupplyCurrent();
    feedback_pending = 6;
  }

  void check_feedback() {
    if (--feedback_pending == 0) {
      feedback_publisher.publish(feedback);
    }
  }

  void voltages(float drive, float supply, float analog) {
    feedback.internal_voltage = drive;
    feedback.supply_voltage = supply;
    feedback.adc_voltage = analog;
    check_feedback();
  }

  void motorCurrent(float current_1, float current_2) {
    feedback.motor_current.resize(2);
    feedback.motor_current[0] = current_1;
    feedback.motor_current[1] = current_2;
    check_feedback();
  }

  void supplyCurrent(float current) {
    feedback.supply_current = current;
    check_feedback();
  }

  void motorCommanded(float commanded_1, float commanded_2) {
    feedback.motor_commanded.resize(2);
    feedback.motor_commanded[0] = commanded_1;
    feedback.motor_commanded[1] = commanded_2;
    check_feedback();
  }

  void encoderRPM(uint32_t rpm_1, uint32_t rpm_2) {
    feedback.encoder_rpm.resize(2);
    feedback.encoder_rpm[0] = rpm_1;
    feedback.encoder_rpm[1] = rpm_2;
    check_feedback();
  }

  void motorPower(float power_1, float power_2) {
    feedback.motor_power.resize(2);
    feedback.motor_power[0] = power_1;
    feedback.motor_power[1] = power_2;
    check_feedback();
  }

  void versionID(std::string ID) {
    ROS_INFO("ROBOTEQ: Version recieved-->%s",ID.c_str());
  }

  uint8_t status_pending;
  ros::Publisher status_publisher;
  roboteq_msgs::Status status;

  void request_status() {
    controller->getFault();
    controller->getStatus();
    controller->getDriverTemperature();
    controller->getMotorTemperature();
    //controller->getDigitalInputs();
    status_pending = 3;
  }

  void check_status() {
    //ROS_INFO("pending at %d",status_pending-1);
    if (--status_pending == 0) {
      status_publisher.publish(status);
    }
  }

  void encoderCount(uint32_t ticks_1, uint32_t ticks_2) {
    check_status();
  }

  // Status message handlers
  void controllerStatus(uint8_t c_status) {
    status.status=c_status;
    check_status();
  }

  void controllerFault(uint8_t fault) {
    status.fault=fault;
    check_status();
  }

  void motorTemperature(float m_temperature) {
    status.motor_temperature.resize(2);
    status.motor_temperature[0]=m_temperature/10;
    check_status();
  }

  void driverTemperature(int temperature_ic,int temperature_chan1,int temperature_chan2) {
    status.channel_temperature.resize(2);
    status.channel_temperature[0]=temperature_chan1;
    status.channel_temperature[1]=temperature_chan2;
    status.ic_temperature=temperature_ic;
    check_status();
  }

public:
  Callbacks(ros::Publisher feedback_publisher2, ros::Publisher status_publisher) :
    feedback_pending(0),
    feedback_publisher(feedback_publisher2),
    status_pending(0),
    status_publisher(status_publisher) {}

  void tick(const ros::TimerEvent&) {
    static uint8_t count = 0;

    if (++count >= 10) {
      count = 0;
    }

    if (controller->connected()) {
      // Every 2nd count, trigger feedback.
      if ((count & 1) == 0) request_feedback();

      // Every 10th count (offset), trigger status.
      if (count == 1) request_status();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string port;
  int32_t baud;
  nh.param<std::string>("port", port, "/dev/ttyACM0");
  nh.param<int32_t>("baud", baud, 115200);

  // Message publishers, and the callbacks to receive serial messages from Roboteq.
  Callbacks callbacks(
    nh.advertise<roboteq_msgs::Feedback>("feedback", 10),
    nh.advertise<roboteq_msgs::Status>("status", 10));

  // Timer is 100Hz so that the 10Hz Status messages can be out of phase from
  // the 50Hz Feedback messages, and minimize the likelihood of jitter.
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), &Callbacks::tick, &callbacks);

  // Message subscribers.
  ros::Subscriber sub_cmd = nh.subscribe("cmd", 1, command_callback);
  ros::Subscriber sub_config = nh.subscribe("config", 1, config_callback);

  // Process ROS IO in background thread.
  ros::AsyncSpinner ros_spinner(1);

  // Serial interface to motor controller.
  controller = new roboteq::Interface(port.c_str(), baud, &callbacks);

  while (ros::ok()) {
    try {
      ROS_DEBUG("Attempting connection to %s at %i.", port.c_str(), baud);
      controller->connect();
      ros_spinner.start();
      while (ros::ok()) {
        controller->spinOnce();
      }
      ros_spinner.stop();
    } catch(roboteq::BadConnection e) {
      ROS_WARN("Problem connecting to serial device. Trying again in 1s.");
      sleep(1);
    }
    
  }

  return 0;
}
