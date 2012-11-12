
#include "roboteq/Callbacks.h"
#include <stdlib.h>


namespace roboteq {


bool Callbacks::handle(string response)
{  
    // ROS_DEBUG("Response: %s", response.c_str());

    uint8_t equals_sign = response.find("=");
    string code = response.substr(0, equals_sign);
    // ROS_DEBUG("  code = %s \n", code);

    // Split up response into multiple fields. 
    response = response.substr(equals_sign + 1);
    size_t delimiter;
    uint8_t i = 0;
	string fields[20];
    while (delimiter != string::npos)  
    {
        delimiter = response.find(":");
        string field = response.substr(0, delimiter).c_str();
        // ROS_DEBUG("  field[%d] = %s \n", i, field);
        fields[i++] = field;
        response = response.substr(delimiter + 1);
    }

    return call(code, fields);
}

float Callbacks::to_float(string field, float scale)
{
    return atoi(field.c_str()) * scale;
}

int32_t Callbacks::to_int(string field)
{
    return atoi(field.c_str());
}


bool Callbacks::call(string code, string fields[])
{
    // Decide what handler to call. Use a switch for single-character
    // codes, and a cascaded if for the others.
    if (code.length() == 1) {
        switch (code[0]) {
            case 'A': motorCurrent(to_float(fields[0]), to_float(fields[1])); break;
            case 'E': closedLoopError(to_int(fields[0])); break;
            case 'M': motorCommanded(to_float(fields[0]), to_float(fields[1])); break;
            case 'C': encoderCount(to_float(fields[0]), to_float(fields[1])); break;
            case 'S': motorRPM(to_float(fields[0]), to_float(fields[1])); break; 
            case 'P': motorPower(to_float(fields[0]), to_float(fields[1])); break;
            case 'V': voltages(to_float(fields[0]), to_float(fields[1]), to_float(fields[2], 0.001)); break;
            default:
                //ROS_WARN("Unhandled code: %s", code);
                return false;
        }
    } else if (code.compare("FID") == 0) {

    } else if (code.compare("VAR") == 0) {
        //userVariable();
    } else if (code.compare("BA") == 0) {
        batteryCurrent(to_float(fields[0])); 
    } else {
        // ROS_WARN("Unhandled code: %s", code)
        return false;
    }
    return true;
}

}
