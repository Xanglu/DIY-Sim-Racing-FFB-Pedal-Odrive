#pragma once

#include "Arduino.h"

static const uint16_t JOYSTICK_MIN_VALUE = 0;
static const uint16_t JOYSTICK_MAX_VALUE = UINT16_MAX;
static const uint16_t JOYSTICK_RANGE = JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE;
uint16_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput);
#ifdef USB_JOYSTICK
#include "TinyusbJoystick.h"
bool IsControllerReady();
void SetupController();
void SetControllerOutputValueBrake(int16_t value);
void SetControllerOutputValueAccelerator(int16_t value);
void SetControllerOutputValueThrottle(int16_t value);
void SetControllerOutputValueRudder(int16_t value);
void SetControllerOutputValueRudder_brake(int16_t value, int16_t value2);
void joystickSendState();
#endif
//bool GetJoystickStatus();
//void RestartJoystick();

