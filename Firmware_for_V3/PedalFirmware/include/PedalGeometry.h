#pragma once

#include "DiyActivePedal_types.h"
#include <Kalman.h>

class StepperWithLimits;

float sledPositionInMM(StepperWithLimits* stepper, DAP_config_st * config_st, float motorRevolutionsPerStep_fl32);
float sledPositionInMM_withPositionAsArgument(float currentPos_fl32, DAP_config_st * config_st, float motorRevolutionsPerStep_fl32);
float pedalInclineAngleDeg(float sledPositionMM, DAP_config_st * config_st);
float pedalInclineAngleAccel(float pedalInclineAngleDeg_global);
float convertToPedalForce(float F_l, float sledPositionMM, DAP_config_st * config_st);
float convertToPedalForceGain(float sledPositionMM, DAP_config_st * config_st);


