#include "roboteq/Interface.h"
#include "roboteq/Callbacks.h"
#include "roboteq/exceptions.h"


double last_path_error_callback=0;

namespace roboteq {

Interface::Interface (const char *port, int baud, Callbacks* callbacks)
    : callbacks_(callbacks), port_(port), baud_(baud), connected_(false), version_("")
{
}

Interface::~Interface()
{
}

void Interface::connect()
{
	char byteRead=0;
	string command_Readback="";
	string version_Readback="";
	int tries=0;

	controllerPort = new LightweightSerial(port_, baud_);

	while(tries<2) {
		if (controllerPort->write_block("?FID\r",5)) {
		
			if (controllerPort->is_ok())
				connected_ = true;
			else {
				connected_ = false;
                throw BadConnection(); 
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
            throw BadConnection();
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
		throw BadConnection();
	}
}


void Interface::spinOnce()
{
    // Process serial messages until there are no more characters waiting in the buffer.
    while(readSerial());
}


bool Interface::readSerial()
{
	string response = "";
	char byteRead = 0;

	if (controllerPort->read(&byteRead)) {
		response += byteRead;
	}else{
		// Nothing to process; exit the function.
        return false;
    }
    	
    while ((int)byteRead != ASCII_CR_CODE)
    {	
        if (controllerPort->read(&byteRead)) {
            response += byteRead;
        } else {
            usleep(100);
        }
    }

	if (response[0] == '-')
	{
        ROS_WARN("CONTROL: Query error-response:>%s", response.c_str());		
        return true;
    } 
    else if (response[0] == '+')
    {
        ROS_DEBUG("CONTROL: Query ACKED send>  response:>%s", response.c_str());
	    return true;
	}
    else
    {
        // Call user callback with data.
        return callbacks_->handle(response);
	}
}


void Interface::sendSerial(string strQuery)
{
	if (!controllerPort->write_block(strQuery.c_str(),strQuery.length())) {
        throw BadTransmission();
	}
}


void Interface::setMotorSpeeds()
{
	string motor_command="";
	/*stringstream m1_string;
	stringstream m2_string;

	m1_string << motor_speed_[0];
	m2_string << motor_speed_[0];
	motor_command = "!M " + m1_string.str() + " " + m2_string.str() + "\r";*/
	sendSerial(motor_command);
}


void Interface::resetDIOx(int i)
{
	string strCommand;
	stringstream stringID;
	stringID << i;
	 strCommand="!D0 " + stringID.str() +"\r";
	sendSerial(strCommand);
}  // resetDIO

void Interface::setDIOx(int i)	
{
    // need to redo this one
	string strCommand;
	stringstream stringID;
	stringID << i;
	strCommand="!D1 " + stringID.str() +"\r";
	sendSerial(strCommand);
}  // setDIO

void Interface::setSetpoint(int motor, int val)
{
    // need to redo this one
	string strCommand;
	stringstream stringMotor;
	stringstream stringVal;
	stringMotor << motor;
	stringVal << val;
	strCommand="!G " + stringMotor.str() + " " + stringVal.str() +"\r";
	last_command_sent_=strCommand;
	sendSerial(strCommand);
}  // setSetpoint

void Interface::setSetpoint(int val)
{
    // need to redo this one
	string strCommand;
	stringstream stringVal;
	stringVal << val;
	strCommand="!G " + stringVal.str() +"\r";
	sendSerial(strCommand);
}  // setSetpoint

void Interface::setEstop()
{
    // need to redo this one
	string strCommand;
	strCommand="!EX \r";
	sendSerial(strCommand);
}  // setEstop

void Interface::resetEstop()
{
    // need to redo this one
	string strCommand;
	strCommand="!MG \r";
	sendSerial(strCommand);
}

/*void Interface::setVAR(int i, int val)
{
    // need to redo this one
	string strCommand;
	stringstream stringID;
	stringstream stringVal;
	stringID << i;
	stringVal << val;
	strCommand="!VAR " + stringID.str() + " " + stringVal.str() +"\r";
	sendSerial(strCommand);
}  // setVAR


void Interface::readVAR(int i)
{
    //need to do this one
	string strQuery;
	stringstream stringID;
	string stringVal;
	stringID << i;
	strQuery="?VAR " + stringID.str() +"\r";
	sendSerial(strQuery);
}  // readVAR
*/

}  // namespace roboteq

