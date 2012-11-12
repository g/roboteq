#include "roboteq/Interface.h"

bool setup_controllers();
//bool sendRuntimeQuery(string strQuery, string *response);


double last_path_error_callback=0;

namespace roboteq {


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
/*	Interface *temp;
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


	motor_controller_1 = new Interface(MCNTRL_PORT_1,115200);
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

	motor_controller_2 = new Interface(MCNTRL_PORT_2,115200);
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
Interface::Interface () 
{
	serialPort = 0;
	controller_baud_ = 0;
	com_status_ = 0;
	motor_speed_[0]=0;
	motor_speed_[1]=0;
	callbackstackpointer=0;
}

Interface::Interface (const char *port, int baud) 
{
	serialPort = port;
	controller_baud_ = baud;
	com_status_ = FALSE; //BAD
	version="";
	callbackstackpointer=0;
}


Interface::~Interface()
{

}

bool Interface::setupComm()
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
			ROS_INFO("CONTROL: Starting Reader thread");
			startInternalThread();
			
		}	
		else {
			ROS_INFO("CONTROL: could not write to Interface");
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

bool Interface::setMotorSpeeds()
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


bool Interface::getUserVariable()
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


bool Interface::getMotorCurrent()
{

	//addCallback(&Interface::decodeMotorCurrent);
	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeMotorCurrent;
	callbackstackpointer++;
	if(sendRuntimeQuery("?A\r")==true)
	{
		return true;	
	}
	else {
		callbackstackpointer--;
		ROS_INFO("CONTROL: Can't get Motor currents failed to send command");
		return false;
	}

}

bool Interface::getBatCurrent()
{
	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeBatCurrent;
	callbackstackpointer++;
	if(sendRuntimeQuery("?BA\r")==true)
	{
		return true;	
	}
	else {
		callbackstackpointer--;
		ROS_INFO("CONTROL: Can't get Motor currents failed to send command");
		return false;
	}

}

bool Interface::getVoltages()
{

	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeVoltage;
	callbackstackpointer++;

	if(sendRuntimeQuery("?V\r")==true)
	{
		return true;
	}
	else{
		callbackstackpointer--;
		ROS_INFO("CONTROL: Can't get Motor and Bat Voltages failed to send command");		
		return false;
	}		
}

bool Interface::sendRuntimeCommand(string strCommand)
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

bool Interface::decodeVoltage(string *value)
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

bool Interface::decodeMotorCurrent(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		motor_current_[0] = atoi(value[0].c_str());
		motor_current_[1] = atoi(value[1].c_str());
		motor_current_[0]/=10.0;
		motor_current_[1]/=10.0;
		//ROS_INFO("Motor current is 1: %f,  2: %f",motor_current_[0],motor_current_[1]);

	return true;		
}

bool Interface::decodeCommanded(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		motorCommanded[0] = atof(value[0].c_str());
		motorCommanded[1] = atof(value[1].c_str());
		motorCommanded[0]/=10.0;
		motorCommanded[1]/=10.0;
		//ROS_INFO("Motor Commanded to  1: %f,  2: %f",motorCommanded[0],motorCommanded[1]);

	return true;		
}

bool Interface::decodeBatCurrent(string *value)
{

	//ROS_INFO("CONTROL:  Decoding voltages value 1 is");
	
		batCurrent = atof(value[0].c_str());
		batCurrent/=10.0;
	
		//ROS_INFO("Battery Current is  %f",batCurrent);

	return true;		
}

bool Interface::decodeMotorRPM(string *value)
{

	//ROS_INFO("CONTROL:  Decoding Motor RPM");	
		encoderRPM[0] = atoi(value[0].c_str());
		encoderRPM[1] = atoi(value[1].c_str());
	
		//ROS_INFO("Encoder Values are %d\t%d", encoderRPM[0],encoderRPM[1]);
	return true;		
}

bool Interface::decodeMotorPower(string *value)
{

	//ROS_INFO("CONTROL:  Decoding Motor RPM");	
		motor_power_[0] = atoi(value[0].c_str());
		motor_power_[1] = atoi(value[1].c_str());
	
		//ROS_INFO("Encoder Values are %d\t%d", encoderRPM[0],encoderRPM[1]);
	return true;		
}		



bool Interface::decodeMotorSetpoint(string *value)
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
/*bool Interface::sendRuntimeQuery(string strQuery, string *response)
{

	//string[] result;
	string InterfaceResponse="";
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
				InterfaceResponse += byteRead;
			}
		}

		
		if (InterfaceResponse[0]!='-')
		{
			
			//ROS_INFO("CONTROL: Query ACKED send> %s : response:>%s",strQuery.c_str(),InterfaceResponse.c_str());
			InterfaceResponse=InterfaceResponse.substr(InterfaceResponse.find("=")+1);
			while (nextLoc!=string::npos)
			{
				nextLoc=InterfaceResponse.find(":");
				//ROS_INFO("for %d got:%s \n",i,InterfaceResponse.substr(0,nextLoc).c_str());
				response[i++]=InterfaceResponse.substr(0,nextLoc).c_str();
				InterfaceResponse=InterfaceResponse.substr(nextLoc+1);
			}

			return 0;
		}
		else
		{
			ROS_INFO("CONTROL: Query error- send> %s : response:>%s",strQuery.c_str(),InterfaceResponse.c_str());		
			return -1;
		}
	}
	else
	{
		ROS_INFO("CONTROL: Query send error");
		return -1;
	}
}*///RuntimeQuery

bool Interface::sendRuntimeQuery(string strQuery)
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
void Interface::readserialbuss()
{
	string response[20];
	string InterfaceResponse="";
	string command_Readback="";
	char byteRead=0;
	int nextLoc=0,i=0;	

	//try waiting and writing back 
	while (1)
	{
		byteRead=0;
		InterfaceResponse="";
				
		for (i=0;i<20;i++)
		{
			response[i]="-1";
		}


		//ROS_INFO("readthread got: %s !!!!: ","ahh");
	
		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);

			if (controllerPort->read(&byteRead)) {
				command_Readback += byteRead;
			}
		}
		//ROS_INFO("Command readback got:%s \n",Readback.substr(0,nextLoc).c_str());
		byteRead=0;
		while ((int)byteRead!=ASCII_CR_CODE)
		{	
			usleep(100);
			if (controllerPort->read(&byteRead)) {
				InterfaceResponse += byteRead;
			}
		}
		if (InterfaceResponse[0]!='-')
		{
		
			if (InterfaceResponse[0]!='+')
			{
				nextLoc=0;
				i=0;
			
					//ROS_INFO("CONTROL: Query ACKED send>  response:>%s",InterfaceResponse.c_str());
					InterfaceResponse=InterfaceResponse.substr(InterfaceResponse.find("=")+1);
					while (nextLoc!=string::npos)
					{
						nextLoc=InterfaceResponse.find(":");
						//ROS_INFO("for %d got:%s \n",i,InterfaceResponse.substr(0,nextLoc).c_str());
						response[i++]=InterfaceResponse.substr(0,nextLoc).c_str();
						InterfaceResponse=InterfaceResponse.substr(nextLoc+1);
					}

					//send data to callback
					//exicute callback from pointer array.
			
					//(this->*callback[0])(response);
					runCallback(response);
			}else{
				//it was a command response send ok.
				//ROS_INFO("CONTROL: Query ACKED send>  response:>%s",InterfaceResponse.c_str());
				response[0]='1';
				runCallback(response);
			}

			
		}else{
			ROS_INFO("CONTROL: Query error-response:>%s",InterfaceResponse.c_str());		
			//return -1;
		}
		//ROS_INFO("readthread got: %s !!!!: Query send error",InterfaceResponse.c_str());
	}	//return NULL;
	return;
}
/*static void *readserialLaunch(void *context)
{
	return((Interface*)context)->readserialbuss(NULL);
}*/
	//bool runCallback(string *data);
	//bool addCallback(void (*func)(void));

bool Interface::addCallback(bool (*func)(void))
{

//	ROS_INFO("CONTROL: added callback @ postion %d",callbackstackpointer);
//	callback[callbackstackpointer]=&Interface::func;
//	callbackstackpointer++;

return true;
}//addcallback

bool Interface::runCallback(string *data)
{
	int i;
	is_callback_locked_=true;
//	ROS_INFO("CONTROL: stat is %s %s %s",data[0].c_str(),data[1].c_str(),data[2].c_str());
	if (callbackstackpointer>0)
	{
	//	ROS_INFO("CONTROL: running function at location 0 stackpointer is at %d",callbackstackpointer);
		//assuming we get serial data in order.
		(this->*callback[0])(data);

		for (i=0;i<callbackstackpointer-1;i++)
		{
			//ROS_INFO("CONTROL: Moviing func from  %d to %d ",i+1,i);
			callback[i]=callback[i+1];
		}
		//remove the last value coppied from the que.	
		callback[i]=NULL;
		callbackstackpointer--;
		is_callback_locked_=false;
		return true;
	}else{
		ROS_INFO("CONTROL: error in callback function callback does not exists!! pointer at %d",callbackstackpointer);
		is_callback_locked_=false;		
	return false;
	}
}//runcallback


bool Interface::startInternalThread()
{
	return(pthread_create(&read_thread_,NULL,InternalThreadEntryFunc,this)==0);//(Interface*)context)->readserialbuss(NULL);
}

static void *InternalThreadEntryFunc(void * This) 
{
	((Interface *)This)->readserialbuss(); 
	return NULL;
}

//Runtime Commands
bool Interface::resetDIOx(int i)
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	 strCommand="!D0 " + stringID.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//resetDIO

bool Interface::setDIOx(int i)	
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setDIO

bool Interface::setSetpoint(int motor,int val)
{
//need to redo this one
	string strCommand;
	stringstream stringMotor;
	stringstream stringVal;
	stringMotor << motor;
	stringVal << val;
	strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setSetpoint

bool Interface::setSetpoint(int val)
{
//need to redo this one
	string strCommand;
	stringstream stringVal;
	stringVal << val;
	strCommand="!G " + stringVal.str() +"\r";
	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeMotorSetpoint;
	callbackstackpointer++;
	if(sendRuntimeQuery(strCommand)==true)
	{
		return true;	
	}
	else {
		callbackstackpointer--;
		ROS_INFO("CONTROL: Can't Set Motor Setpoint failed to send command");
		return false;
	}



return sendRuntimeQuery(strCommand);
}//setSetpoint

bool Interface::setEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!EX \r";
	return sendRuntimeCommand(strCommand);
}//setEstop

bool Interface::resetEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!MG \r";
	return sendRuntimeCommand(strCommand);
}

bool Interface::setVAR(int i, int val)
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringstream stringVal;
	stringID << i;
	stringVal << val;
	strCommand="!VAR " + stringID.str() + " " + stringVal.str() +"\r";
	return sendRuntimeCommand(strCommand);
}//setVAR


int Interface::readVAR(int i)
{
//need to do this one
	string qryResponse[20]; 
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	if(sendRuntimeQuery(strQuery)==0)
	{
		//ROS_INFO("String Val %d: %s",i,stringVal.c_str());
	//	return atoi(qryResponse[i].c_str());
			
	}
	else
		return -1;
}//readVAR



bool Interface::getClosedLoopError()
{
//need to do this one


	if(sendRuntimeQuery("?E\r")==true)
	{

	//	closedLoopErr = atof(Error.c_str());
		
	//	ROS_INFO("Closed loop error is: %f",closedLoopErr);

		return true;
	}
	else
		return false;

}//get closed loop error


	
bool Interface::getMotorCommanded()
{

	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeCommanded;
	callbackstackpointer++;

	if(sendRuntimeQuery("?M\r")==true)
	{
		return true;
	}
	else
		ROS_INFO("CONTROL: Can't get Motor Commanded failed to send command");
		
		callbackstackpointer--;
		return false;

}//get motor commanded


bool Interface::getEncoderCount()
{
	//need to do this one


	if(sendRuntimeQuery("?C\r")==true)
	{

		//encoderCount[0] = atol(EncoderCountValues[0].c_str());
		//encoderCount[1] = atol(EncoderCountValues[1].c_str());
		
		//ROS_INFO("Motor Encoders to: 1: %f,2: %f",EncoderCountValues[0],EncoderCountValues[1]);

		return true;
	}
	else
		return false;

}//get encoder count


bool Interface::getMotorRPM()
{
	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeMotorRPM;
	callbackstackpointer++;

	if(sendRuntimeQuery("?S\r")==true)
	{
		return true;
	}else{
		callbackstackpointer--;
		return false;
	}
}//get motor RPM

bool Interface::getMotorPower()
{
	
	while(is_callback_locked_);
	callback[callbackstackpointer]=&Interface::decodeMotorPower;
	callbackstackpointer++;

	if(sendRuntimeQuery("?P\r")==0)
	{
		return true;
	}
	else
		return false;

}//get motor RPM



//need to do this one!!!
bool Interface::getFault()
{
	

	if(sendRuntimeQuery("?S\r")==0)
	{

		return true;
	}
	else
		return false;

}//get motor RPM

	bool getFault();
	bool getStatus();

    }

