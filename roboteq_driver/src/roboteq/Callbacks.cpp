
#include "roboteq/Callbacks.h"
#include <stdlib.h>
#include "ros/ros.h"

namespace roboteq {


bool Callbacks::handle(string response)
{  
   //  ROS_INFO("Response: %s", response.c_str());

    size_t equals_sign = response.find("=");
    if (equals_sign == string::npos)
    {
        // Line which includes no data.
        return false;
    }
    string code = response.substr(0, equals_sign);

    // Split up response into multiple fields. 
    response = response.substr(equals_sign + 1);
    size_t delimiter = 0;
    uint8_t i = 0;
	string fields[20];
    while (delimiter != string::npos)  
    {
        delimiter = response.find(":");
        string field = response.substr(0, delimiter).c_str();
        // ROS_INFO("  field[%d] = %s \n", i, field.c_str());
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
            case 'S': encoderRPM(to_float(fields[0]), to_float(fields[1])); break; 
            case 'P': motorPower(to_float(fields[0]), to_float(fields[1])); break;
            case 'V': voltages(to_float(fields[0]), to_float(fields[1]), to_float(fields[2], 0.001)); break;
			case 'T': driverTemperature(to_int(fields[0]), to_int(fields[1]), to_int(fields[2]));break;
            default:
                //ROS_WARN("Unhandled code: %s", code);
                return false;
        }
    } else if (code.compare("FID") == 0) {
		versionID(fields[0]);
    } else if (code.compare("VAR") == 0) {
        //userVariable();
    } else if (code.compare("BA") == 0) {
        supplyCurrent(to_float(fields[0])); 
	} else if (code.compare("FS") == 0) {
        controllerStatus(to_int(fields[0]));
	} else if (code.compare("FF") == 0) {
        controllerFault(to_int(fields[0])); 
	} else if (code.compare("AI") == 0) {
        motorTemperature(to_float(fields[0])); //only getting one  
    } else {

        // ROS_WARN("Unhandled code: %s", code)
        return false;
    }
    return true;
}

}
