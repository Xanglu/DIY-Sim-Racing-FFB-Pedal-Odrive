/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2021 NeKuNeKo for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "Adafruit_TinyUSB.h"

/* This sketch demonstrates USB HID gamepad use.
 * This sketch is only valid on boards which have native USB support
 * and compatibility with Adafruit TinyUSB library.
 * For example SAMD21, SAMD51, nRF52840.
 *
 * Make sure you select the TinyUSB USB stack if you have a SAMD board.
 * You can test the gamepad on a Windows system by pressing WIN+R, writing Joy.cpl and pressing Enter.
 */

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

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(3000000);

  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.println("Adafruit TinyUSB HID Single Axis example");
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif
  
  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

  if (!usb_hid.ready()) return;

  // Ramp up the axis value
  axis_value += 50;

  // For CDC instance 0 (the default one)
  bool dtr = tud_cdc_connected();       // true if host has opened the port (DTR asserted)
  bool rts = tud_cdc_get_line_state() & 2; // bit1 = RTS, bit0 = DTR

  if (dtr) {
    Serial.print("Sending axis value: ");
    Serial.println(axis_value);
  }
  usb_hid.sendReport(0, &axis_value, sizeof(axis_value));
}