#ifndef ROBOTEQ_CALLBACKS
#define ROBOTEQ_CALLBACKS

#include <stdint.h>
#include <string>

using namespace std;

namespace roboteq {

class NoHandler : public exception
{
};

class Callbacks
{
  public:
    bool handle(string);

  protected:
    void motorCurrent(float current_1, float current_2) { throw NoHandler(); }
    void batteryCurrent(float battery) { throw NoHandler(); }
    void closedLoopError(uint8_t error) { throw NoHandler(); }
    void motorCommanded(float commanded_1, float commanded_2) { throw NoHandler(); }
    void encoderCount(uint32_t ticks_1, uint32_t ticks_2) { throw NoHandler(); }
    void motorRPM(uint32_t rpm_1, uint32_t rpm_2) { throw NoHandler(); }
    void motorPower(float power_1, float power_2) { throw NoHandler(); }
    void voltages(float drive, float battery, float analog) { throw NoHandler(); }

  private:
    bool call(string code, string fields[]);
    static float to_float(string field, float scale=0.1);
    static int32_t to_int(string field);
};

}

#endif
