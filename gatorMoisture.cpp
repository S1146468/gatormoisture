/**
* Andy England @ SparkFun Electronics
* September 6, 2018
* https://github.com/sparkfun/pxt-light-bit
*
* Development environment specifics:
* Written in Microsoft PXT
* Tested with a SparkFun temt6000 sensor and micro:bit
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/


#include "gatorMoisture.h"
#include "pxt.h"
#include <cstdint>
#include <math.h>

using namespace pxt;
using namespace gatorMoisture;

//void InverseKinamaticModel(float* output_array, float x, float y, float z, float pitch, float roll, float yaw, int leg_ID);
//void InverseKinamaticModel(float* output_array, float x, float y, float z, float pitch, float roll, float yaw, int leg_ID);
//float getMoisture(int16_t ADCVal);

namespace gatorMoisture {
    /*
    * Calculates the light in Lux based on the ADC value passed in. 1 step in adcVal is equal to .488 uA or .976 lux at 5V
    */
    //%
    float getMoisture(int16_t ADCVal) {
        return ADCVal / 1023.0f;
    }

    //%
    void InverseKinamaticModel()
    {

    }
}


