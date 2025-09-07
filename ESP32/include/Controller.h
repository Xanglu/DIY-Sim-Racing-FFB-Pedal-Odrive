#pragma once

#include "Arduino.h"
#include "Main.h"

static const uint16_t JOYSTICK_MIN_VALUE = 0;
static const uint16_t JOYSTICK_MAX_VALUE = 0xFFFF;
static const uint16_t JOYSTICK_RANGE = JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE;

int32_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput);



#ifdef USB_JOYSTICK
  void SetupController_USB(uint8_t pedal_ID);
  bool IsControllerReady();
  void SetControllerOutputValue(int32_t value);
#endif