#include "roboteq/roboteq.h"

bool setup_controllers();
//bool SendSerial(string strQuery, string *response);


double last_path_error_callback=0;




/*int main(int argc, char **argv)
{

	int i=0;

	ros::init(argc, argv, "igvc_control");
	ros::NodeHandle n;
	igvc_control::control_msg Control_Info;	
	igvc_control::motor_diag Motor_Diagnostics;
	omniwheel=false;
	autonomous=false;
	auto_velControl=false;

	if (!setup_controllers()) {
		ROS_INFO("Motor Controllers Not Setup..");
		return 0;
	}

	ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);
	ros::Subscriber error_sub = n.subscribe("path_errors",1,pathErrorCallback);
	ros::Publisher ControlInfo_pub = n.advertise<igvc_control::control_msg>("Control_Info", 1);
	ros::Publisher MotorDiag_pub = n.advertise<igvc_control::motor_diag>("Motor_State",1);

	ros::Rate loop_rate(50);		//100Hz update rate
	while (ros::ok())
  	{
			
		i++;
		if (i>10) {
			i=0;

			motor_controller_1->getMotorCurrent();
			motor_controller_1->getVoltages();
			motor_controller_2->getMotorCurrent();
			motor_controller_2->getVoltages();

			Motor_Diagnostics.c1m1_batCurrent = motor_controller_1->Motor1Current;
			Motor_Diagnostics.c1m2_batCurrent = motor_controller_1->Motor2Current;
		
			Motor_Diagnostics.c2m1_batCurrent = motor_controller_2->Motor1Current;
			Motor_Diagnostics.c2m2_batCurrent = motor_controller_2->Motor2Current;

			Motor_Diagnostics.c1_batVolt = motor_controller_1->battery_voltage;
			Motor_Diagnostics.header.stamp = ros::Time::now();
			Motor_Diagnostics.c2_batVolt = motor_controller_2->battery_voltage;
			MotorDiag_pub.publish(Motor_Diagnostics);

		}
		//if (autonomous)
			//path_tracker();

		if (autonomous && (ros::Time::now().toSec() - last_path_error_callback>0.5))
		{
			motor_controller_1->Motor1Speed=0;
			motor_controller_1->Motor2Speed=0;
			motor_controller_2->Motor1Speed=0;
			motor_controller_2->Motor2Speed=0;
		}

		motor_controller_1->setMotorSpeeds();
		motor_controller_2->setMotorSpeeds();	
		Control_Info.Motor1Speed = motor_controller_1->Motor1Speed;
		Control_Info.Motor2Speed = motor_controller_2->Motor1Speed;
		Control_Info.autonomous = autonomous;
		Control_Info.auto_velControl = auto_velControl;
		Control_Info.rc_omniwheel = omniwheel;
		ControlInfo_pub.publish(Control_Info);
		
		ros::spinOnce();	
		loop_rate.sleep();			//used for maintaining rate
	}


	return 0;
}
*/


bool setup_controllers()
{
/*	roboteq *temp;
	std:string mControlUserVar="";
	K=gsl_matrix_alloc(4,3);
	gsl_matrix_set(K,0,0,1);
	gsl_matrix_set(K,1,0,1);
	gsl_matrix_set(K,2,0,1);
	gsl_matrix_set(K,3,0,1);

	gsl_matrix_set(K,0,1,-1);
	gsl_matrix_set(K,1,1,1);
	gsl_matrix_set(K,2,1,-1);
	gsl_matrix_set(K,3,1,1);

	gsl_matrix_set(K,0,2,-1*(c+l));
	gsl_matrix_set(K,1,2,-1*(c+l));
	gsl_matrix_set(K,2,2,c+l);
	gsl_matrix_set(K,3,2,c+l);


	motor_controller_1 = new roboteq(MCNTRL_PORT_1,115200);
	motor_controller_1->Motor1Speed=0;
	motor_controller_1->Motor2Speed=0;

	if (motor_controller_1->setupComm())
		ROS_INFO("CONTROL: Controller (%s) Setup Complete", MCNTRL_PORT_1);		
	else { 
		ROS_INFO("CONTROL: Controller (%s) Not Setup Properly", MCNTRL_PORT_1);
		return FALSE;
	}
	if (!motor_controller_1->getUserVariable())
		return FALSE;

	motor_controller_2 = new roboteq(MCNTRL_PORT_2,115200);
	motor_controller_2->Motor1Speed=0;
	motor_controller_2->Motor2Speed=0;

	if (motor_controller_2->setupComm())
		ROS_INFO("CONTROL: Controller (%s) Setup Complete", MCNTRL_PORT_2);		
	else {
		ROS_INFO("CONTROL: Controller (%s) Not Setup Properly", MCNTRL_PORT_2);
		return FALSE;
	}

	if (!motor_controller_2->getUserVariable())
		return FALSE;

	mControlUserVar = motor_controller_1->user_var;
	if (mControlUserVar.compare("VAR=0\r")!=0) {
		temp = motor_controller_1;
		motor_controller_1 = motor_controller_2;
		motor_controller_2 = temp;

	}
	ROS_INFO("Motor Controller 1:%s",motor_controller_1->user_var.c_str());
	ROS_INFO("Motor Controller 2:%s",motor_controller_2->user_var.c_str());
*/
	return TRUE;
}
roboteq::roboteq () 
{
	serialPort = 0;
	controller_baud_ = 0;
	com_status_ = 0;
	motor_speed_[0]=0;
	motor_speed_[1]=0;

}

roboteq::roboteq (const char *port, int baud) 
{
	serialPort = port;
	controller_baud_ = baud;
	com_status_ = FALSE; //BAD
	version="";

}


roboteq::~roboteq()
{

}

bool roboteq::setupComm()
{
	char byteRead=0;
	string command_Readback="";
	string version_Readback="";
	int tries=0;

	
	controllerPort = new LightweightSerial(serialPort,controller_baud_);

	while(tries<2) {
		if (controllerPort->write_block("?FID\r",5)) {
		
			if (controllerPort->is_ok())
				com_status_=TRUE; //good
			else {
				com_status_ = FALSE;

				return com_status_;
			}

			while ((int)byteRead!=ASCII_CR_CODE)
			{

				usleep(100);
				if (controllerPort->read(&byteRead)) {

						command_Readback += byteRead;
				}
			}
			byteRead=0;
			while ((int)byteRead!=ASCII_CR_CODE)
			{	
				usleep(100);
				if (controllerPort->read(&byteRead)) {
						version_Readback += byteRead;
				}
			}
			ROS_INFO("CONTROL: Version recieved-->%s",version_Readback.c_str());	
		}	
		else {
			ROS_INFO("CONTROL: could not write to roboteq");
			com_status_=FALSE;
			return com_status_;
		}
		
		if (version_Readback.length()<2) { //system not cleared..try one more time
			version_Readback="";
			tries++; 
		}
		else
			tries = 3;
	}
	
	if (version_Readback.length()<2 && tries > 2) { //never got a reply :(
		ROS_INFO("CONTROL: Motor controller not replying");
		com_status_ = FALSE;
	}

	return com_status_;
}

bool roboteq::setMotorSpeeds()
{
	string motor_command="";
	stringstream m1_string;
	stringstream m2_string;
	//string ack="";
	//string command_Readback="";
	//char byteRead=0;


	m1_string << motor_speed_[0];
	m2_string << motor_speed_[0];
	motor_command = "!M " + m1_string.str() + " " + m2_string.str() + "\r";
	return SendSerial(motor_command);

}


bool roboteq::getUserVariable()
{
	string command_Readback="";
	char byteRead=0;

	if (controllerPort->write_block("?VAR\r",5)) {

		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);

			if (controllerPort->read(&byteRead)) {
				command_Readback += byteRead;
			}
		}

		byteRead=0;
		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);
			if (controllerPort->read(&byteRead)) {
				user_var += byteRead;
			}
		}
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't get user variable");
		return false;
	}


//return(0);

}


bool roboteq::getMotorCurrent()
{

	//addCallback(&roboteq::decodeMotorCurrent);

	if(SendSerial("?A\r")==true)
	{
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't get Motor currents failed to send command");
		return false;
	}

}

bool roboteq::getBatCurrent()
{
	if(SendSerial("?BA\r")==true)
	{
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't get Motor currents failed to send command");
		return false;
	}

}

bool roboteq::getVoltages()
{

	if(SendSerial("?V\r")==true)
	{
		return true;
	}
	else{
		ROS_INFO("CONTROL: Can't get Motor and Bat Voltages failed to send command");		
		return false;
	}		
}

bool roboteq::SendSerial(string strCommand)
{

	string ack="";
	string command_Readback="";
	char byteRead=0;

	if (controllerPort->write_block(strCommand.c_str(),strCommand.length())) {
		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);

			if (controllerPort->read(&byteRead)) {
				command_Readback += byteRead;
			}
		}
		byteRead=0;
		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);
			if (controllerPort->read(&byteRead)) {
				ack += byteRead;
			}
		}


		//response=byteRead;
		
		if (ack[0]==('+'))
		{
			//ROS_INFO("CONTROL:  COMMAND ACKED -> %s",strCommand.c_str());
			return 0;
		}
		else
		{
			//ROS_INFO("CONTROL:  Command receive error- > %s",strCommand.c_str());		
			return -1;
		}
	}
	else
	{
		ROS_INFO("CONTROL:  Command send error");
		return -1;
	}
}//RuntimeCommand

bool roboteq::decodeVoltage(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	

		drive_voltage = atoi(value[0].c_str());
		battery_voltage = atoi(value[1].c_str());
		analog_voltage = atoi(value[2].c_str());
		drive_voltage/=10.0;
		battery_voltage/=10.0;
		analog_voltage/=1000.0;
		//ROS_INFO("CONTROL:  Decoding voltages value 1 is %s, 2 is %s, 3 is %s",value[0].c_str(),value[1].c_str(),value[2].c_str());
		//ROS_INFO("Motor voltage: %f, Bat Voltage: %f, Analog voltage: %f",drive_voltage,battery_voltage,analog_voltage);

	return true;		
}

bool roboteq::decodeMotorCurrent(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		motor_current_[0] = atoi(value[0].c_str());
		motor_current_[1] = atoi(value[1].c_str());
		motor_current_[0]/=10.0;
		motor_current_[1]/=10.0;
		//ROS_INFO("Motor current is 1: %f,  2: %f",motor_current_[0],motor_current_[1]);

	return true;		
}

bool roboteq::decodeCommanded(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		motorCommanded[0] = atof(value[0].c_str());
		motorCommanded[1] = atof(value[1].c_str());
		motorCommanded[0]/=10.0;
		motorCommanded[1]/=10.0;
		//ROS_INFO("Motor Commanded to  1: %f,  2: %f",motorCommanded[0],motorCommanded[1]);

	return true;		
}

bool roboteq::decodeBatCurrent(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		batCurrent = atof(value[0].c_str());
		batCurrent/=10.0;
	
		//ROS_INFO("Battery Current is  %f",batCurrent);

	return true;		
}

bool roboteq::decodeMotorRPM(string *value)
{

	//ROS_INFO("CONTROL:  Decoding Motor RPM");	
		encoderRPM[0] = atoi(value[0].c_str());
		encoderRPM[1] = atoi(value[1].c_str());
	
		//ROS_INFO("Encoder Values are %d\t%d", encoderRPM[0],encoderRPM[1]);
	return true;		
}

bool roboteq::decodeMotorPower(string *value)
{

	//ROS_INFO("CONTROL:  Decoding Motor RPM");	
		motor_power_[0] = atoi(value[0].c_str());
		motor_power_[1] = atoi(value[1].c_str());
	
		//ROS_INFO("Encoder Values are %d\t%d", encoderRPM[0],encoderRPM[1]);
	return true;		
}		



bool roboteq::decodeMotorSetpoint(string *value)
{

	//ROS_INFO("CONTROL:  Decoding Motor RPM");	
	if(atoi(value[0].c_str())==1)
	{
		//all is good
		//ROS_INFO("Setpoint Command accepted");
	}else{
		//error
		ROS_INFO("Setpoint Command NOT accepted response was %s",value[0].c_str());
	}
	
		
	return true;		
}


bool roboteq::SendSerial(string strQuery)
{
	if (controllerPort->write_block(strQuery.c_str(),strQuery.length())) {
	
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Query send error");
		return -1;
	}
}//RuntimeQuery

//Reading thread
void roboteq::readserialbuss()
{
	string response[20];
	string roboteqResponse="";
	string command_Readback="";
	char byteRead=0;
	int nextLoc=0,i=0;	
	//ROS_INFO("readthread got: %s !!!!: ","ahh");

	if (controllerPort->read(&byteRead)) {
		command_Readback += byteRead;
	}else{
		//nothing to read exit the function.
		return;
	}

	while ((int)byteRead!=ASCII_CR_CODE)
	{	
		if (controllerPort->read(&byteRead)) {
			command_Readback += byteRead;
		}
		usleep(100);	
	}
	//ROS_INFO("Command readback got:%s \n",Readback.substr(0,nextLoc).c_str());
	byteRead=0;
	while ((int)byteRead!=ASCII_CR_CODE)
	{	
		if (controllerPort->read(&byteRead)) {
			roboteqResponse += byteRead;
		}else{
			usleep(100);
		}
	}
	if (roboteqResponse[0]!='-')
	{
	
		if (roboteqResponse[0]!='+')
		{

		//Send serial response to secondary data thread.
		
		ROS_INFO("CONTROL: Query ACKED send>  response:>%s",roboteqResponse.c_str());
			
		}else{
			//it was a command response do nothing. ok.
			//ROS_INFO("CONTROL: Query ACKED send>  response:>%s",roboteqResponse.c_str());


		}

		
	}else{
		ROS_INFO("CONTROL: Query error response was:%s",roboteqResponse.c_str());		
		//return -1;
	}
}

//Runtime Commands
bool roboteq::resetDIOx(int i)
{
	string strCommand;
	stringstream stringID;
	stringID << i;
	 strCommand="!D0 " + stringID.str() +"\r";
	return SendSerial(strCommand);
}//resetDIO

bool roboteq::setDIOx(int i)	
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	return SendSerial(strCommand);
}//setDIO

bool roboteq::setSetpoint(int motor,int val)
{
//need to redo this one
	string strCommand;
	stringstream stringMotor;
	stringstream stringVal;
	stringMotor << motor;
	stringVal << val;
	strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
	return SendSerial(strCommand);
}//setSetpoint

bool roboteq::setSetpoint(int val)
{
//need to redo this one
	string strCommand;
	stringstream stringVal;
	stringVal << val;
	strCommand="!G " + stringVal.str() +"\r";
	if(SendSerial(strCommand)==true)
	{
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't Set Motor Setpoint failed to send command");
		return false;
	}

return SendSerial(strCommand);
}//setSetpoint

bool roboteq::setEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!EX \r";
	return SendSerial(strCommand);
}//setEstop

bool roboteq::resetEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!MG \r";
	return SendSerial(strCommand);
}

bool roboteq::setVAR(int i, int val)
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringstream stringVal;
	stringID << i;
	stringVal << val;
	strCommand="!VAR " + stringID.str() + " " + stringVal.str() +"\r";
	return SendSerial(strCommand);
}//setVAR


int roboteq::readVAR(int i)
{
//need to do this one
	string qryResponse[20]; 
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	if(SendSerial(strQuery)==0)
	{
		//ROS_INFO("String Val %d: %s",i,stringVal.c_str());
	//	return atoi(qryResponse[i].c_str());
			
	}
	else
		return -1;
}//readVAR



bool roboteq::getClosedLoopError()
{
//need to do this one


	if(SendSerial("?E\r")==true)
	{

	//	closedLoopErr = atof(Error.c_str());
		
	//	ROS_INFO("Closed loop error is: %f",closedLoopErr);

		return true;
	}
	else
		return false;

}//get closed loop error


	
bool roboteq::getMotorCommanded()
{
	if(SendSerial("?M\r")==true)
	{
		return true;
	}
	else
		ROS_INFO("CONTROL: Can't get Motor Commanded failed to send command");
		return false;

}//get motor commanded


bool roboteq::getEncoderCount()
{
	//need to do this one


	if(SendSerial("?C\r")==true)
	{
		return true;
	}
	else
		return false;

}//get encoder count


bool roboteq::getMotorRPM()
{

	if(SendSerial("?S\r")==true)
	{
		return true;
	}else{
		return false;
	}
}//get motor RPM

bool roboteq::getMotorPower()
{

	if(SendSerial("?P\r")==0)
	{
		return true;
	}
	else
		return false;

}//get motor RPM



//need to do this one!!!
bool roboteq::getFault()
{
	

	if(SendSerial("?S\r")==0)
	{

		return true;
	}
	else
		return false;

}//get motor RPM

	bool getFault();
	bool getStatus();

