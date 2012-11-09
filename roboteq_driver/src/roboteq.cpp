#include "roboteq/roboteq.h"

bool setup_controllers();


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






/*void path_tracker()
{
	double omega=0;
	double motor1speed=0;
	double motor2speed=0;
	double reqd_r_wheelspeed=0;
	double reqd_l_wheelspeed=0;
	k_cntrl = 2;
	k_yaw = 2.5; //1.5

	if (!auto_velControl)
		reqd_vel=REQD_VX;

//	reqd_vel=0;
//	omega=-0.5;	
//	reqd_vel=0;


	omega = k_yaw*yaw_e + atan2(k_cntrl*cross_e,reqd_vel);
	reqd_r_wheelspeed = (reqd_vel + c*omega)/r;
	reqd_l_wheelspeed = (reqd_vel - c*omega)/r;

	ROS_INFO("CONTROL:cross track:%f   heading_error:%f    calculated omega:%f",cross_e,yaw_e,omega);

	motor1speed = reqd_l_wheelspeed/(enc_wheel_rev*2*pi/60);
	motor2speed = reqd_r_wheelspeed/(enc_wheel_rev*2*pi/60);

	//Single Motor Controller
//	motor_controller_1->Motor2Speed = motor1speed;
//	motor_controller_1->Motor1Speed = -1*motor2speed;

	//Double Motor Controller
	motor_controller_1->Motor1Speed = M1_MULT*motor1speed;
	motor_controller_1->Motor2Speed = M1_MULT*motor1speed;
	motor_controller_2->Motor1Speed = M2_MULT*motor2speed;
	motor_controller_2->Motor2Speed = M2_MULT*motor2speed;
	
}*/

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
	controllerBaud = 0;
	comStatus = 0;
	Motor1Speed=0;
	Motor2Speed=0;
}

roboteq::roboteq (const char *port, int baud) 
{
	serialPort = port;
	controllerBaud = baud;
	comStatus = FALSE; //BAD
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
	
	controllerPort = new LightweightSerial(serialPort,controllerBaud);
	while(tries<2) {
		if (controllerPort->write_block("?FID\r",5)) {
			if (controllerPort->is_ok())
				comStatus=TRUE; //good
			else {
				comStatus = FALSE;
				return comStatus;
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
			comStatus=FALSE;
			return comStatus;
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
		comStatus = FALSE;
	}

	return comStatus;
}

bool roboteq::setMotorSpeeds()
{
	string motor_command="";
	stringstream m1_string;
	stringstream m2_string;
	//string ack="";
	//string command_Readback="";
	//char byteRead=0;


	m1_string << Motor1Speed;
	m2_string << Motor2Speed;
	motor_command = "!M " + m1_string.str() + " " + m2_string.str() + "\r";
	return sendRuntimeCommand(motor_command);

/*
	if (controllerPort->write_block(motor_command.c_str(),motor_command.length())) {
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

		//if (!ack.compare("+"))
		//	ROS_INFO("CONTROL: Motor command receive error");		

		
		if (ack.compare("+"))
			ROS_INFO("CONTROL: MOTOR COMMAND ACKED -> %s",motor_command.c_str());
		else
			ROS_INFO("CONTROL: Motor command receive error");		
	}
	else
	{
		ROS_INFO("CONTROL: Motor command send error");
	}

	return 0;*/
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
	string current_values[2];
	if(sendRuntimeQuery("?A\r", current_values)==0)
	{
		Motor1Current = atoi(current_values[0].c_str());
		Motor2Current = atoi(current_values[1].c_str());
		Motor1Current/=10.0;
		Motor2Current/=10.0;
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't get Motor currents");
		return false;
	}

}

bool roboteq::getVoltages()
{

	string voltage_values[3];

	if(sendRuntimeQuery("?V\r", voltage_values)==0)
	{

		drive_voltage = atoi(voltage_values[0].c_str());
		battery_voltage = atoi(voltage_values[1].c_str());
		analog_voltage = atoi(voltage_values[2].c_str());
		drive_voltage/=10.0;
		battery_voltage/=10.0;
		analog_voltage/=1000.0;
		//ROS_INFO("Motor voltage: 1: %f,2: %f",drive_voltage,battery_voltage);

		return true;
	}
	else
		return false;

	
	

}

bool roboteq::sendRuntimeCommand(string strCommand)
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

bool roboteq::sendRuntimeQuery(string strQuery, string *response)
{

	//string[] result;
	string roboteqResponse="";
	string command_Readback="";
	char byteRead=0;
	int i=0,nextLoc=0;

	if (controllerPort->write_block(strQuery.c_str(),strQuery.length())) {
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
				roboteqResponse += byteRead;
			}
		}

		
		if (roboteqResponse[0]!='-')
		{
			
			//ROS_INFO("CONTROL: Query ACKED send> %s : response:>%s",strQuery.c_str(),roboteqResponse.c_str());
			roboteqResponse=roboteqResponse.substr(roboteqResponse.find("=")+1);
			while (nextLoc!=string::npos)
			{
				nextLoc=roboteqResponse.find(":");
				//ROS_INFO("for %d got:%s \n",i,roboteqResponse.substr(0,nextLoc).c_str());
				response[i++]=roboteqResponse.substr(0,nextLoc).c_str();
				roboteqResponse=roboteqResponse.substr(nextLoc+1);
			}

			return 0;
		}
		else
		{
			ROS_INFO("CONTROL: Query error- send> %s : response:>%s",strQuery.c_str(),roboteqResponse.c_str());		
			return -1;
		}
	}
	else
	{
		ROS_INFO("CONTROL: Query send error");
		return -1;
	}
}//RuntimeQuery

/*bool roboteq::sendRuntimeQuery(string strQuery, string *response)
{


	string ack="";
	string command_Readback="";
	char byteRead=0;

	if (controllerPort->write_block(strQuery.c_str(),strQuery.length())) {
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


		*response=ack;

		
		if (ack.compare("-"))
		{
			ROS_INFO("CONTROL: Query ACKED send> %s : response:>%s",strQuery.c_str(),ack.c_str());
			return 0;
		}
		else
		{
			ROS_INFO("CONTROL: Query error- send> %s : response:>%s",strQuery.c_str(),ack.c_str());		
			return -1;
		}
	}
	else
	{
		ROS_INFO("CONTROL: Query send error");
		return -1;
	}
}//RuntimeQuery*/

//Runtime Commands
bool roboteq::resetDIOx(int i)
{
	string strCommand;
	stringstream stringID;
	stringID << i;
	 strCommand="!D0 " + stringID.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//resetDIO

bool roboteq::setDIOx(int i)	
{
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setDIO

bool roboteq::setSetpoint(int motor,int val)
{
	string strCommand;
	stringstream stringMotor;
	stringstream stringVal;
	stringMotor << motor;
	stringVal << val;
	strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setSetpoint

bool roboteq::setSetpoint(int val)
{
	string strCommand;
	stringstream stringVal;
	stringVal << val;
	strCommand="!G " + stringVal.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setSetpoint

bool roboteq::setEstop()
{
	string strCommand;
	strCommand="!EX \r";
	return sendRuntimeCommand(strCommand);
}//setEstop

bool roboteq::resetEstop()
{
	string strCommand;
	strCommand="!MG \r";
	return sendRuntimeCommand(strCommand);
}

bool roboteq::setVAR(int i, int val)
{
	string strCommand;
	stringstream stringID;
	stringstream stringVal;
	stringID << i;
	stringVal << val;
	strCommand="!VAR " + stringID.str() + " " + stringVal.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setVAR


int roboteq::readVAR(int i)
{
	string qryResponse[20]; 
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	if(sendRuntimeQuery(strQuery, qryResponse)==0)
	{
		//ROS_INFO("String Val %d: %s",i,stringVal.c_str());
		return atoi(qryResponse[i].c_str());
			
	}
	else
		return -1;
}//readVAR


//bool roboteq::getBatteryAmps();






/*



	
	
	

	//Runtime Querys	
	--bool roboteq::getMotorCurrent();
	--bool roboteq::getAnalogValues();
	--bool roboteq::getAnalogValue(int i);
	--bool roboteq::getBatteryAmps();
	--bool roboteq::getEncoderCountABS();
	--bool roboteq::getEncoderCountREL();
	--bool roboteq::getDigitalInputs();
	//do we need individual inputs?
	--bool roboteq::getDigitalOutputs();
	bool roboteq::getClosedLoopError();
	bool roboteq::getFeedbackIn();
	--bool roboteq::getFault();
	--bool roboteq::getStatus();
	--bool roboteq::getMotorPowerOutput();
	--bool roboteq::getPulsedInputs();
	--bool roboteq::getSpeeds();
	--bool roboteq::getTemp();
	--bool roboteq::getVolts();
	bool roboteq::readVAR();
	bool roboteq::controllerPresent();*/

