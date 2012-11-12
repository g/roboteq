#include "ros/ros.h"
#include "roboteq/Interface.h"
#include "roboteq/Callbacks.h"
#include "geometry_msgs/Twist.h"


roboteq::Interface* controller;

void request_feedback(const ros::TimerEvent&)
{
	controller->getMotorCurrent();
	controller->getBatteryCurrent();
	controller->getMotorCommanded();
	controller->getMotorRPM();
	controller->getMotorPower();
}

void request_status(const ros::TimerEvent&)
{
	controller->getVoltages();
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
};
	/*ROS_INFO("Motor Drive Current 1: %f\t2: %f ",motor_controller_FR->motor_current_[0],motor_controller_FR->motor_current_[1]);
	ROS_INFO("Bat Current %f ",motor_controller_FR->batCurrent);
	ROS_INFO("Motor Comanded 1: %f\t2: %f ",motor_controller_FR->motorCommanded[0],motor_controller_FR->motorCommanded[1]);
	ROS_INFO("Motor RPM 1: %d\t2: %d ",motor_controller_FR->encoderRPM[0],motor_controller_FR->encoderRPM[1]);
	ROS_INFO("Motor Power 1: %d\t2: %d ",motor_controller_FR->motor_power_[0],motor_controller_FR->motor_power_[1]);
*/   


int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_driver");
	ros::NodeHandle n;
	Callbacks callbacks;

	//Publishers subscribers here
	
	//Subscribe to twist messages for commanded velocity.
	// ros::Subscriber commandedTwist = n.subscribe("/husky/cmd_vel", 100, setMotorSpeedsFromControler);
	//rospy.Subscriber("plan_cmd_vel", Twist, self.planned_callback)
	//roboteq motor_controller_RR;
	//motor_controller setup here
	//motor_controller_RR = new roboteq("/dev/ttyACM0", 115200);  // Front left controller
	
	controller = new roboteq::Interface("/dev/ttyACM2", 115200, &callbacks);  // Front left controller
	//roboteq* motor_controller_FL = new roboteq("/dev/ttyACM1", 115200);  // Front right controller
	//roboteq* motor_controller_RR = new roboteq("/dev/ttyACM2", 115200);  // Front right controller
	//roboteq* motor_controller_RL = new roboteq("/dev/ttyACM3", 115200);  // Front right controller
	
	controller->connect();

    // 50 Hz timer for Feedback data.
	ros::Timer timer_feedback = n.createTimer(ros::Duration(0.02), request_feedback);
    
    // 10 Hz timer for Status data.
	ros::Timer timer_status = n.createTimer(ros::Duration(0.1), request_status);

    while (true)
	{
		ros::spinOnce();
		controller->spinOnce();
        usleep(100);
	}

	return 0;
}


	
