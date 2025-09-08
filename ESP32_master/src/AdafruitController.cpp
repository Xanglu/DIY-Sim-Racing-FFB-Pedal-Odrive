#include "AdafruitController.h"
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

// HID Report Descriptor for Racing Pedal (Brake only)

uint8_t const desc_hid_report[] = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x04, // Usage (Joystick)
    0xA1, 0x01, // Collection (Application)

    // Define six 16-bit axes (X, Y, Z, Rx, Ry, Rz)
    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x09, 0x35,       //   Usage (Rz)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0xFF, //   Logical Maximum (65535)
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x06,       //   Report Count (6) <-- 這裡從2改為6
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0xC0, // End Collection
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload for the two axes
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t rx;
    uint16_t ry;
    uint16_t rz;
} hid_report_t;

hid_report_t hid_report = {0};

void SetupController()
{
    
    // Set VID and PID
    Serial.println("[L]starting USB joystick");
    TinyUSBDevice.setID(0x3035, 0x8213);
    TinyUSBDevice.setProductDescriptor("DIY_FFB_PEDAL_JOYSTICK");
    TinyUSBDevice.setManufacturerDescriptor("OPENSOURCE");
    //ActiveSerial->
    // Manual begin() is required on core without built-in support e.g. mbed rp2040
    if (!TinyUSBDevice.isInitialized())
    {
        TinyUSBDevice.begin(0);
    }

    // Setup HID
    usb_hid.setPollInterval(8); // time in ms
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.begin();

    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted())
    {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }
}

bool IsControllerReady()
{
    bool returnValue_b = true;
    if (!TinyUSBDevice.mounted())
    {
        returnValue_b = false;
    }
    if (!usb_hid.ready())
    {
        returnValue_b = false;
    }

    return returnValue_b;
}

void SetControllerOutputValueBrake(int16_t value)
{

    uint16_t tmp = value;

    if (tmp < 0x7FFF)
    {
        tmp += 0x7FFF + 1;
    }
    else
    {
        tmp -= 0x7FFF + 1;
    }

    hid_report.rx = tmp;
    usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}

void SetControllerOutputValueAccelerator(int16_t value)
{

    uint16_t tmp = value;

    if (tmp < 0x7FFF)
    {
        tmp += 0x7FFF + 1;
    }
    else
    {
        tmp -= 0x7FFF + 1;
    }

    hid_report.ry = tmp;
    usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}

void SetControllerOutputValueThrottle(int16_t value)
{

    uint16_t tmp = value;

    if (tmp < 0x7FFF)
    {
        tmp += 0x7FFF + 1;
    }
    else
    {
        tmp -= 0x7FFF + 1;
    }

    hid_report.rz = tmp;
    usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}

void SetControllerOutputValueRudder(int16_t value)
{

    uint16_t tmp = value;

    if (tmp < 0x7FFF)
    {
        tmp += 0x7FFF + 1;
    }
    else
    {
        tmp -= 0x7FFF + 1;
    }

    hid_report.x = tmp;
    usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}
void SetControllerOutputValueRudder_brake(int16_t value, int16_t value2)
{

    uint16_t tmp = value;
    uint16_t tmp2 = value;
    if (tmp < 0x7FFF)
    {
        tmp += 0x7FFF + 1;
    }
    else
    {
        tmp -= 0x7FFF + 1;
    }
    if (tmp2 < 0x7FFF)
    {
        tmp2 += 0x7FFF + 1;
    }
    else
    {
        tmp2 -= 0x7FFF + 1;
    }
    hid_report.y = tmp;
    hid_report.z = tmp;
    usb_hid.sendReport(0, &hid_report, sizeof(hid_report));
}
