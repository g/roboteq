#ifndef ROBOTEQ_CALLBACKS
#define ROBOTEQ_CALLBACKS

#include <stdint.h>
#include <string>
#include <roboteq/exceptions.h>

using namespace std;

namespace roboteq {

class Callbacks
{
  private:
    virtual void motorCurrent(float current_1, float current_2) { throw NoHandler(); }
    virtual void supplyCurrent(float battery) { throw NoHandler(); }
    virtual void closedLoopError(uint8_t error) { throw NoHandler(); }
    virtual void motorCommanded(float commanded_1, float commanded_2) { throw NoHandler(); }
    virtual void encoderCount(uint32_t ticks_1, uint32_t ticks_2) { throw NoHandler(); }
    virtual void encoderRPM(uint32_t rpm_1, uint32_t rpm_2) { throw NoHandler(); }
    virtual void motorPower(float power_1, float power_2) { throw NoHandler(); }
    virtual void voltages(float drive, float battery, float analog) { throw NoHandler(); }
	virtual void versionID(string ID){throw NoHandler();}
	virtual void controllerStatus(uint8_t c_status){throw NoHandler();}
	virtual void controllerFault(uint8_t fault){throw NoHandler();}
	virtual void controllerTemperatue(uint8_t fault){throw NoHandler();}
	virtual void motorTemperature(float m_temperature){throw NoHandler();}
	virtual void driverTemperature(int temperature_ic,int temperature_chan1,int temperature_chan2){throw NoHandler();}

    /*virtual void logdebug(string s);
    virtual void loginfo(string s);
    virtual void logwarn(string s);*/

    // Internal helper functions
    bool call(string code, string fields[]);
    static float to_float(string field, float scale=0.1);
    static int32_t to_int(string field);

  public:
    bool handle(string);
};

}

#endif
