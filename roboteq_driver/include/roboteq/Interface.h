#ifndef ROBOTEQ_INTERFACE
#define ROBOTEQ_INTERFACE

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roboteq/lightweightserial.h"
#include "roboteq/Callbacks.h"
#include <unistd.h>
#include <iostream>
#include <sstream>

#define TRUE 1
#define FALSE 0

#define ASCII_CR_CODE 13
#define ASCII_ACK_CODE 43

#define ROBOTEQ_MAX_SETPOINT 1000 //This is defined by the roboteq software range is from -1000 to +1000

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
	int sendSerialBlocking(string strQuery, string response);

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

	//Configuration Parameters, when writing these values should be confirmed (aka should probably block)
	
	int setMotorAmpLimit(float amp_limit);
	int setMotorAmpLimit(uint8_t channel,float amp_limit);
	int setPIDICap(uint8_t channel, float pid_cap);
	int setPIDKi(float ki_1,float ki_2);
	int setPIDKp(uint8_t channel,float kp);
	int setPIDkd(float kd_1,float kd_2);
	int setMaxAcc(float max_acc_1,float max_acc_2);
	int setMaxAcc(float max_acc);
	int setMaxDec(float max_dcc);
	int setMaxDec(float max_dcc_1,float max_dec_2);
	int setOperatingMode(uint8_t mode_1,uint8_t mode_2);
	int setMaxPwrFwd(float max_fwd_1,float max_fwd_2);
	int setMaxPwrFwd(float max_fwd);
	int setMaxPwrRev(float max_rev_1,float max_rev_2);
	int setMaxPwrRev(float max_rev_1);
	int setMaxRPM(uint16_t max_rpm_1,uint16_t max_rpm_2);
	int setMaxRPM(uint16_t max_rpm);
	int setOvervoltageLimit(float overvoltage_limit);
	int setUndervoltageLimit(float undervoltage_limit);
	int setEncoderPulsePerRev(uint16_t encoder_pulse_per_rev_1,uint16_t encoder_pulse_per_rev_2);
	int setSerialEcho(bool serial_echo);
	int setSerialWatchdogTimeout(float wd_timeout);
	int setAutomaticTelemetry(string telem_request);
	int writeValuesToEEPROM();


	
};

}

#endif
