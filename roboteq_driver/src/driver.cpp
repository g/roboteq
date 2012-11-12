#include "ros/ros.h"
#include "roboteq/Interface.h"
#include "geometry_msgs/Twist.h"


#define LIN_MAX 1000
#define ANG_MAX 1000
#define l 2	//need to put the real value here.


roboteq::Interface* controller;
volatile int globali=0;


void setMotorSpeedsFromControler(const geometry_msgs::Twist::ConstPtr& msg)
 {
	float ls_vel,rs_vel;
	float norm_vel,norm_ang; 
 	ROS_INFO("I heard: (%lf,%lf)", msg->linear.x,msg->angular.z);
	
	norm_vel=msg->linear.x;
	norm_ang=msg->angular.z;

	norm_vel=norm_vel/sqrt(norm_vel*norm_vel+norm_ang*norm_ang);
	norm_ang=norm_ang/sqrt(norm_vel*norm_vel+norm_ang*norm_ang);

	//ls_vel=norm_vel*LIN_MAX - norm_ang*ANG_MAX;
	//rs_vel=norm_vel*LIN_MAX + norm_ang*ANG_MAX;
	ls_vel=msg->linear.x*LIN_MAX - msg->angular.z*ANG_MAX;
	rs_vel=msg->linear.x*LIN_MAX + msg->angular.z*ANG_MAX;

	ROS_INFO("LS: %lf,  RS: %lf",ls_vel,rs_vel);
 }

void test_function(const ros::TimerEvent&)
{
	//ROS_INFO("Getting Voltages");
	controller->getVoltages();
	controller->getMotorCurrent();
	controller->getBatCurrent();
	controller->getMotorCommanded();
	controller->getMotorRPM();
	controller->getMotorPower();
	


/*motor_controller_FR->getVoltages();
motor_controller_FR->getVoltages();
motor_controller_FR->getVoltages();
motor_controller_FR->getVoltages();

*/	
	//ROS_INFO("Getting Current");
	
	/*ROS_INFO("Motor Drive Current 1: %f\t2: %f ",motor_controller_FR->motor_current_[0],motor_controller_FR->motor_current_[1]);
	ROS_INFO("Motor Drive voltage %f  Bat Voltage %f analog Voltage %f",motor_controller_FR->drive_voltage,motor_controller_FR->battery_voltage,motor_controller_FR->analog_voltage);
	ROS_INFO("Bat Current %f ",motor_controller_FR->batCurrent);
	ROS_INFO("Motor Comanded 1: %f\t2: %f ",motor_controller_FR->motorCommanded[0],motor_controller_FR->motorCommanded[1]);
	ROS_INFO("Motor RPM 1: %d\t2: %d ",motor_controller_FR->encoderRPM[0],motor_controller_FR->encoderRPM[1]);
	ROS_INFO("Motor Power 1: %d\t2: %d ",motor_controller_FR->motor_power_[0],motor_controller_FR->motor_power_[1]);

*/
//globali++;
	//motor_controller_FR->setVAR(1, globali);


}

void test_function2(const ros::TimerEvent&)
{
	//ROS_INFO("Wuddup 2");
	controller->setSetpoint(globali++);
//	ROS_INFO("Wuddup 2 ,var = %d", motor_controller_FR->readVAR(1));

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "thor_driver");
	ros::NodeHandle n;
	
	//Publishers subscribers here
	
	//Subscribe to twist messages for commanded velocity.
	ros::Subscriber commandedTwist = n.subscribe("/husky/cmd_vel", 100, setMotorSpeedsFromControler);
	//rospy.Subscriber("plan_cmd_vel", Twist, self.planned_callback)
	//roboteq motor_controller_RR;
	//motor_controller setup here
	//motor_controller_RR = new roboteq("/dev/ttyACM0", 115200);  // Front left controller
	
	controller = new roboteq::Interface("/dev/ttyACM2", 115200);  // Front left controller
	//roboteq* motor_controller_FL = new roboteq("/dev/ttyACM1", 115200);  // Front right controller
	//roboteq* motor_controller_RR = new roboteq("/dev/ttyACM2", 115200);  // Front right controller
	//roboteq* motor_controller_RL = new roboteq("/dev/ttyACM3", 115200);  // Front right controller
	
	controller->setupComm();
	//motor_controller_FL->setupComm();
	//roboteq::roboteq("ttyACM0", 115200) 

	

	ros::Rate loop_rate (1);

	//Setup Serial Port

	//setup timers for getting data from roboteq
	ros::Timer timer = n.createTimer(ros::Duration(.015), test_function);
	ros::Timer timer2 = n.createTimer(ros::Duration(.5), test_function2);
	//ros::Timer timer3 = n.createTimer(ros::Duration(2), test_function2);

	ros::spin();

	return 0;
}


	
