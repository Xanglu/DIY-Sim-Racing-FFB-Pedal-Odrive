#pragma once

#include "Arduino.h"
#include "Main.h"

static const int16_t JOYSTICK_MIN_VALUE = 0;
static const int16_t JOYSTICK_MAX_VALUE = 10000;
static const int16_t JOYSTICK_RANGE = JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE;

int32_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput);



#ifdef USB_JOYSTICK
  void SetupController();
  void SetupController_USB(uint8_t pedal_ID);
  bool IsControllerReady();

  void SetControllerOutputValue(int32_t value);
  void SetControllerOutputValue_rudder(int32_t value, int32_t value2);

  void JoystickSendState();
  bool GetJoystickStatus();
  void RestartJoystick();

#endif