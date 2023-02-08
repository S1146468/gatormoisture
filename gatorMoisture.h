#ifndef GATORMOISTURE_H_
#define GATORMOISTURE_H_

#include "pxt.h"
#include <cstdint>
#include <math.h>

using namespace pxt;

export namespace gatorMoisture {
	int InverseKinamaticModel();
	//void InverseKinamaticModel(float* output_array, float x, float y, float z, float pitch, float roll, float yaw, int leg_ID);
	void getMoisture(int16_t ADCVal);
}

#endif