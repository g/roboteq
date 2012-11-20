#ifndef ROBOTEQ_INTERFACE
#define ROBOTEQ_INTERFACE

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboteq/lightweightserial.h"
#include "roboteq/Callbacks.h"
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <pthread.h>

#define TRUE 1
#define FALSE 0

#define MCNTRL_PORT_1 "/dev/ttyACM0"
#define MCNTRL_PORT_2 "/dev/ttyACM1"


#define ASCII_CR_CODE 13
#define ASCII_ACK_CODE 43

using namespace std;

namespace roboteq {

class Interface {
  private :
    Callbacks* callbacks_;
	const char *port_;
	int baud_;
	bool connected_;
	std::string version_;
	LightweightSerial *controllerPort;
	
	void sendSerial(string strQuery);
    bool readSerial();
	std::string last_command_sent_;

  public :
    Interface (const char *port, int baud, Callbacks* callbacks);
	~Interface();
	void connect();
	bool connected() { return connected_; }
	void spinOnce();

	void setMotorSpeeds();
	void resetDIOx(int i);
	void setDIOx(int i);
	void setSetpoint(int motor,int val);
	void setSetpoint(int val);
	void setEstop();
	void resetEstop();
	void setVAR(int i, int val);

	// Runtime Querys.
	void getMotorCurrent() { sendSerial("?A\r"); }
	void getSupplyCurrent() { sendSerial("?BA\r"); }
	void getEncoderCount() { sendSerial("?C\r"); }
	void getEncoderRPM() { sendSerial("?S\r"); }
	void getDigitalInputs();
	void getDigitalOutputs();
	void getClosedLoopError() { sendSerial("?E\r"); }
	void getFeedbackIn();
	void getFault() { sendSerial("?FF\r"); }
	void getStatus() { sendSerial("?FS\r"); }
    void getMotorPower() { sendSerial("?P\r"); }
	void getPulsedInputs();
	void getDriverTemperature(){sendSerial("?T\r");}
	void getMotorTemperature(){sendSerial("?AI 1\r");}
	void getMotorCommanded() { sendSerial("?M\r"); }
	void getUserVariable() { sendSerial("?VAR\r"); }
	void getVoltages() { sendSerial("?V\r"); }
};

}

#endif
