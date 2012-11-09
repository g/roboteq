#ifndef ROBOTEQ_SERIAL_CONTROLLER
#define ROBOTEQ_SERIAL_CONTROLLER

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboteq/lightweightserial.h"
#include <unistd.h>
#include <iostream>
#include <sstream>

//#include "igvc_control/control_msg.h" //for messages
//#include "igvc_control/motor_diag.h"	

//#include "igvc_control/lightweightserial.h"
//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_linalg.h>
//#include <gsl/gsl_blas.h>
//#include "geometry_msgs/Vector3.h"
//#include "joy/Joy.h"

#define TRUE 1
#define FALSE 0

#define MCNTRL_PORT_1 "/dev/ttyACM0"
#define MCNTRL_PORT_2 "/dev/ttyACM1"

#define MAX_SPEED 1800.0
#define MAX_YAW 1800.0

#define max_yaw_rate 20 //rad/s
#define max_fwd_vel 18 //m/s

#define ASCII_CR_CODE 13
#define ASCII_ACK_CODE 43
/*
#define M1_MULT 1
#define M2_MULT -1

//Vehicle Parameters
#define c 0.385 //m half of the wheel base
#define l 0.4318 //m half of length
#define enc_wheel_rev 0.185 // (14/60) rpm conversion factor between encoder RPM's and wheel RPM's
#define r 0.13 //m wheel radius
#define x0 0.1 //x-coordinate of ICR (assuming constant slippage)
#define pi 3.14159

#define REQD_VX 0.5 //target forward velocity m/s
#define DEFAULT_K_YAW 4
#define DEFAULT_K_CNTRL 0.1
*/

using namespace std;

//bool autonomous=false;
//bool omniwheel=false;
//bool auto_velControl = false;
//double k_cntrl=DEFAULT_K_YAW; //controller gain
//double k_yaw = DEFAULT_K_CNTRL;; //yaw error controller gain
//void joystickCallback(const joy::Joy::ConstPtr& msg);
//void pathErrorCallback(const geometry_msgs::Vector3 msg);


//GSL Matrix Abstractions
//gsl_matrix* matrix_inverse(gsl_matrix *A);
//gsl_matrix* matrix_multiply(gsl_matrix *A, gsl_matrix *B);
//static void matrix_print(gsl_matrix *A);

//Nonlinear Path Tracker Variables
//void path_tracker();
//double yaw_e = 0; //current yaw error
//double cross_e=0; //currrent cross_track error
//double reqd_vel=0;


class roboteq {
	private :
	const char *serialPort;
	int controllerBaud;
	bool comStatus;
	std::string version;
		
	LightweightSerial *controllerPort;
	bool sendRuntimeCommand(string strCommand);
	bool sendRuntimeQuery(string strQuery, string *response);
	public :
	int Motor1Speed;
	int Motor2Speed;
	double Motor1Current;
	double Motor2Current;
	double drive_voltage;
	double battery_voltage;
	double analog_voltage;
	int encoder1Count;
	int encoder2Count;
	int statusFlag;
	int fautFlag;


	std::string user_var;
	roboteq();
	roboteq(const char *port, int baud);
	~roboteq();
	bool setupComm();
	bool setMotorSpeeds();
	bool getUserVariable();
	
	bool getVoltages();

	
	//To impliment
	//Runtime Commands
	bool resetDIOx(int i);
	bool setDIOx(int i);
	bool setSetpoint(int motor,int val);
	bool setSetpoint(int val);
	bool setEstop();
	bool resetEstop();
	bool setVAR(int i, int val);
	

	//Runtime Querys	
	bool getMotorCurrent();
	bool getAnalogValues();
	bool getAnalogValue(int i);
	bool getBatteryAmps();
	bool getEncoderCountABS();
	bool getEncoderCountREL();
	bool getDigitalInputs();
	//do we need individual inputs?
	bool getDigitalOutputs();
	bool getClosedLoopError();
	bool getFeedbackIn();
	bool getFault();
	bool getStatus();
	bool getMotorPowerOutput();
	bool getPulsedInputs();
	bool getSpeeds();
	bool getTemp();
	bool getVolts();
	int readVAR();
	int readVAR(int i);
	bool controllerPresent();
	
	
	
	
	

	

};





//roboteq* motor_controller_1; //left controller
//roboteq* motor_controller_2; //right controller

#endif
