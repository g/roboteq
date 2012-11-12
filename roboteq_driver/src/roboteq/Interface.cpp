#include "roboteq/Interface.h"
#include "roboteq/Callbacks.h"


double last_path_error_callback=0;

namespace roboteq {

Interface::Interface (const char *port, int baud, Callbacks* callbacks)
    : callbacks_(callbacks), port_(port), baud_(baud)
{
	com_status_ = FALSE; //BAD
	version="";
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

	
	controllerPort = new LightweightSerial(port_ , baud_);

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
	return sendSerial(motor_command);

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
	if(sendSerial("?A\r")==true)
	{
		return true;	
	}
	else {
		ROS_INFO("CONTROL: Can't get Motor currents failed to send command");
		return false;
	}

}

bool Interface::getBatCurrent()
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

bool Interface::getVoltages()
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
}

//RuntimeCommand
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

bool Interface::sendSerial(string strQuery)
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
	string response="";
	string command_Readback="";
	char byteRead=0;

	if (controllerPort->read(&byteRead)) {
		command_Readback += byteRead;
	}else{
		//nothing to read exit the function.
		return;
	}

	byteRead=0;
	while ((int)byteRead!=ASCII_CR_CODE)
	{	
		if (controllerPort->read(&byteRead)) {
			response += byteRead;
		}else{
			usleep(100);
			if (controllerPort->read(&byteRead)) {
				response += byteRead;
			}
		}
    }

	if (response[0] == '-')
	{
        ROS_WARN("CONTROL: Query error-response:>%s", response.c_str());		
    } 
    else if (response[0] == '+')
    {
        ROS_DEBUG("CONTROL: Query ACKED send>  response:>%s", response.c_str());
	}
    else
    {
        // Call user callback with data.
        callbacks_->handle(response);
	}
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


//Runtime Commands
bool Interface::resetDIOx(int i)
{
	string strCommand;
	stringstream stringID;
	stringID << i;
	 strCommand="!D0 " + stringID.str() +"\r";
	return sendSerial(strCommand);
}//resetDIO

bool Interface::setDIOx(int i)	
{
//need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	return sendSerial(strCommand);
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
	last_command_sent_=strCommand;
	return sendSerial(strCommand);
}//setSetpoint

bool Interface::setSetpoint(int val)
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

bool Interface::setEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!EX \r";
	return sendSerial(strCommand);
}//setEstop

bool Interface::resetEstop()
{
//need to redo this one
	string strCommand;
	strCommand="!MG \r";
	return sendSerial(strCommand);
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
	return sendSerial(strCommand);
}//setVAR


int Interface::readVAR(int i)
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


bool Interface::getClosedLoopError()
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
	
bool Interface::getMotorCommanded()
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


bool Interface::getEncoderCount()
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


bool Interface::getMotorRPM()
{
	if(sendSerial("?S\r")==true)
	{
		return true;
	}else{
		return false;
	}
}//get motor RPM

bool Interface::getMotorPower()
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


bool Interface::getFault()
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

bool Interface::getStatus()
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


    }

