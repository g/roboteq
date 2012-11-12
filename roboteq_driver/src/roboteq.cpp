#include "roboteq/roboteq.h"


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
	return sendSerial(motor_command);

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

	if(sendSerial("?A\r")==true)
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
	if(sendSerial("?BA\r")==true)
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

	if(sendSerial("?V\r")==true)
	{
		return true;
	}
	else{
		ROS_INFO("CONTROL: Can't get Motor and Bat Voltages failed to send command");		
		return false;
	}		
}

bool roboteq::sendSerial(string strQuery)
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
	//string response[20];
	string roboteqResponse="";
	string command_Readback="";
	char byteRead=0;
	//int nextLoc=0,i=0;	
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
	return sendSerial(strCommand);
}//resetDIO

bool roboteq::setDIOx(int i)	
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	return sendSerial(strCommand);
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
	last_command_sent_=strCommand;
	return sendSerial(strCommand);
}//setSetpoint

bool roboteq::setSetpoint(int val)
{
//need to redo this one
	string strCommand;
	stringstream stringVal;
	stringVal << val;
	strCommand="!G " + stringVal.str() +"\r";
	if(sendSerial(strCommand)==true)
	{
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't Set Motor Setpoint failed to send command");
		return false;
	}

return sendSerial(strCommand);
}//setSetpoint

bool roboteq::setEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!EX \r";
	return sendSerial(strCommand);
}//setEstop

bool roboteq::resetEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!MG \r";
	return sendSerial(strCommand);
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
	return sendSerial(strCommand);
}//setVAR

int roboteq::readVAR(int i)
{
//need to do this one
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	if(sendSerial(strQuery)==0)
	{
		return true;		
	}
	else
		return -1;
}//readVAR

bool roboteq::getClosedLoopError()
{
	if(sendSerial("?E\r")==true)
	{
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Closed Loop Error failed to send command");
		return false;
	}

}//get closed loop error
	
bool roboteq::getMotorCommanded()
{
	if(sendSerial("?M\r")==true)
	{
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Motor Commanded failed to send command");
		return false;
	}

}//get motor commanded

bool roboteq::getEncoderCount()
{
	//need to do this one
	if(sendSerial("?C\r")==true)
	{
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Encoder count failed to send command");
		return false;
	}

}//get encoder count

bool roboteq::getMotorRPM()
{

	if(sendSerial("?S\r")==true)
	{
		return true;
	}else{
		return false;
	}
}//get motor RPM

bool roboteq::getMotorPower()
{

	if(sendSerial("?P\r")==true)
	{
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Motor Power failed to send command");
		return false;
	}

}//get motor RPM

bool roboteq::getFault()
{
	if(sendSerial("?FF\r")==0)
	{

		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Roboteq Fault failed to send command");
		return false;
	}

}//get roboteq Faults

bool roboteq::getStatus()
{
	if(sendSerial("?FS\r")==0)
	{
		return true;
	}
	else
	{
		ROS_INFO("CONTROL: Can't get Roboteq Status failed to send command");
		return false;
	}

}//get roboteq Status


