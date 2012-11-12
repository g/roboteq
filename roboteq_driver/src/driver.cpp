#include "roboteq/Interface.h"
#include "roboteq/Callbacks.h"
#include "roboteq/exceptions.h"

#include "ros/ros.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Status.h"
#include "roboteq_msgs/Feedback.h"


roboteq::Interface* controller;

void request_feedback(const ros::TimerEvent&)
{
    if (controller->connected()) {
    	controller->getMotorCurrent();
	    controller->getBatteryCurrent();
    	controller->getMotorCommanded();
	    controller->getMotorRPM();
	    controller->getMotorPower();
    }
}

void request_status(const ros::TimerEvent&)
{
    if (controller->connected()) {
	    controller->getVoltages();
    }
}

void command_callback(const roboteq_msgs::Command& command)
{


}


class Callbacks : public roboteq::Callbacks {
  private:
    void voltages(float drive, float battery, float analog) {
	    ROS_INFO("Motor Drive voltage %f  Bat Voltage %f analog Voltage %f",
                drive, battery, analog);
    }
    
    void motorCurrent(float current_1, float current_2) {
	    ROS_INFO("Motor Drive Current 1: %f\t2: %f ",
               current_1, current_2);
    }

  public:
    roboteq_msgs::Feedback feedback;
    roboteq_msgs::Status status;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "~");
	ros::NodeHandle nh;

    string port;
    int32_t baud;
    nh.param<std::string>("port", port, "/dev/ttyACM0");
    nh.param<int32_t>("baud", baud, 115200);

	Callbacks callbacks;
    controller = new roboteq::Interface(port.c_str(), baud, &callbacks);
    
    // 50 Hz timer for Feedback data.
	ros::Timer timer_feedback = nh.createTimer(ros::Duration(0.02), request_feedback);
    
    // 10 Hz timer for Status data.
	ros::Timer timer_status = nh.createTimer(ros::Duration(0.1), request_status);

    while (ros::ok()) {
        try {
            ROS_DEBUG("Attempting connection to %s at %i.", port.c_str(), baud);
	        controller->connect();

            while (ros::ok())
	        {
		        ros::spinOnce();
		        controller->spinOnce();
                usleep(100);
	        }
        } catch(roboteq::BadConnection e) {
            ROS_WARN("Problem connecting to serial device. Trying again in 1s.");
            sleep(1);
        }
    }

	return 0;
}


