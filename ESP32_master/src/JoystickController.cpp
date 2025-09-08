#include "JoystickController.h"

//#include "Main.h"
uint16_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput)
{
    float valRange = (maxVal - minVal);
    if (abs(valRange) < 0.01)
    {
        return JOYSTICK_MIN_VALUE; // avoid div-by-zero
    }

    float fractional = (value - minVal) / valRange;
    uint16_t controller = JOYSTICK_MIN_VALUE + (maxGameOutput / 100.0f) * (fractional * JOYSTICK_RANGE);
    return controller;
}


#ifdef USB_JOYSTICK
TinyusbJoystick tinyusbJoystick_;
void SetupController()
{
    
    tinyusbJoystick_.Begin();
    
}

bool IsControllerReady()
{
    bool returnValue_b = tinyusbJoystick_.IsReady();
    return returnValue_b;
}

void SetControllerOutputValueBrake(int16_t value)
{
    tinyusbJoystick_.SetRx(value);
}

void SetControllerOutputValueAccelerator(int16_t value)
{
    tinyusbJoystick_.SetRy(value);
}

void SetControllerOutputValueThrottle(int16_t value)
{
    tinyusbJoystick_.SetRz(value);
}

void SetControllerOutputValueRudder(int16_t value)
{
    tinyusbJoystick_.SetXAxis(value);
}
void SetControllerOutputValueRudder_brake(int16_t value, int16_t value2)
{

    uint16_t tmp = value;
    uint16_t tmp2 = value;
    tinyusbJoystick_.SetYAxis(value);
    tinyusbJoystick_.SetZAxis(value2);
}

void joystickSendState()
{
    tinyusbJoystick_.SendState();
}
#endif

