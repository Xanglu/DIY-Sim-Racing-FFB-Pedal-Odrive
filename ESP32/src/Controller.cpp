#include "Controller.h"

uint16_t NormalizeControllerOutputValue(float value, float minVal, float maxVal, float maxGameOutput) {
  float valRange_fl32 = (maxVal - minVal);
  if (fabsf(valRange_fl32) < 0.01f) {
    return JOYSTICK_MIN_VALUE;   // avoid div-by-zero
  }

  float corrected_range_fl32 = 0.005f * valRange_fl32;
  float corrected_min_value_fl32 = minVal + corrected_range_fl32;
  float corrected_max_value_fl32 = maxVal - corrected_range_fl32;
  float corrected_valRange_fl32 = (corrected_max_value_fl32 - corrected_min_value_fl32);

  float fractional_fl32 = (value - corrected_min_value_fl32) / corrected_valRange_fl32;
  int32_t controller_i32 = JOYSTICK_MIN_VALUE + (fractional_fl32 * JOYSTICK_RANGE);
  int16_t controller_u16 = constrain(controller_i32, JOYSTICK_MIN_VALUE, (maxGameOutput * 0.01f) * JOYSTICK_MAX_VALUE);
  return controller_u16;
}


#ifdef USB_JOYSTICK

#include <string>
#include "Adafruit_TinyUSB.h"

// HID Report Descriptor for Racing Pedal (Brake only)
const uint8_t desc_hid_report[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Controls)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)

    0x05, 0x02,        //   Usage Page (Simulation Controls)
    0x09, 0xC5,        //   Usage (Brake)  <-- special "pedal" usage
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //   Logical Maximum (65535)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs)  <-- absolute, not relative

    0xC0               // End Collection
};


// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload for the two axes
typedef struct {
  int16_t brake;
} hid_report_t;

hid_report_t hid_report = {0};



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

void SetupController() 
{

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

void SetControllerOutputValue(uint16_t value) {
  
  // trafo to sint16, as report seems to expect that
  int16_t tmp = map(value, 0, UINT16_MAX, INT16_MIN, INT16_MAX);

  hid_report.brake = tmp;
  usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}



#endif
