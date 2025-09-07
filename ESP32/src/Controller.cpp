#include "Controller.h"

int32_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput) {
  float valRange = (maxVal - minVal);
  if (abs(valRange) < 0.01) {
    return JOYSTICK_MIN_VALUE;   // avoid div-by-zero
  }

  float fractional = (value - minVal) / valRange;
  int32_t controller = JOYSTICK_MIN_VALUE + (fractional * JOYSTICK_RANGE);
  return constrain(controller, JOYSTICK_MIN_VALUE, (maxGameOutput/100.) * JOYSTICK_MAX_VALUE);
}


#ifdef USB_JOYSTICK

#include <string>
#include "Adafruit_TinyUSB.h"

// HID report descriptor
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] =
{
  0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x09, 0x30,        //   Usage (X)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xFF, 0xFF,  //   Logical Maximum (65535)
  0x75, 0x10,        //   Report Size (16)
  0x95, 0x01,        //   Report Count (1)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0xC0,              // End Collection
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload
uint16_t axis_value = 0;

int32_t previousTransmittedControllerValue_u32 = 0;
bool newControllerValueReceived_b = false;


void SetupController_USB(uint8_t pedal_ID) 
{
  int PID;
  char *APname;
  switch(pedal_ID)
  {
    case 0:
      PID=0x8214;
      APname="FFB_Pedal_Clutch";
      break;
    case 1:
      PID=0x8215;
      APname="FFB_Pedal_Brake";
      break;
    case 2:
      PID=0x8216;
      APname="FFB_Pedal_Throttle";
      break;
    default:
      PID=0x8217;
      APname="FFB_Pedal_NOASSIGNMENT";
      break;

  }

    // Set VID and PID
  TinyUSBDevice.setID(0x3035, PID);
  TinyUSBDevice.setProductDescriptor(APname);
  TinyUSBDevice.setManufacturerDescriptor("OpenSource");

  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // Setup HID
  usb_hid.setPollInterval(10); // time in ms
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

bool IsControllerReady() { 
  bool returnValue_b = true;
  if (!TinyUSBDevice.mounted()) {
    returnValue_b = false;
  }
  if (!usb_hid.ready())
  {
    returnValue_b = false;
  }

  return returnValue_b;
}

void SetControllerOutputValue(int32_t value) {
  
  axis_value = value;

  previousTransmittedControllerValue_u32 = value;
  newControllerValueReceived_b = true;
  usb_hid.sendReport(0, &axis_value, sizeof(axis_value));
}



#endif