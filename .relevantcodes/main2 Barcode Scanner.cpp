#include <SPI.h>
#include <hidboot.h>
#include <usbhub.h>

USB Usb;
USBHub Hub(&Usb);
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> Keyboard(&Usb);

String barcode = "";  // String to hold the barcode data

class KbdRptParser : public KeyboardReportParser {
  void PrintKey(uint8_t mod, uint8_t key);

 protected:
  void OnKeyDown(uint8_t mod, uint8_t key);
  void OnKeyPressed(uint8_t key);
};

void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key) {
  uint8_t c = OemToAscii(mod, key);

  if (c) OnKeyPressed(c);
}

void KbdRptParser::OnKeyPressed(uint8_t key) {
  if (key == '\r' || key == '\n') {  // Check for carriage return or newline
    if (barcode.length() > 0) {
      Serial.println(
          barcode);  // Print the complete barcode followed by a newline
      barcode = "";  // Clear the barcode string for the next input
    }
  } else {
    barcode += (char)key;  // Append character to barcode string
  }
}

KbdRptParser Prs;

void setup() {
  Serial.begin(9600);
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed.");
    while (1);  // Halt
  }
  delay(200);
  Keyboard.SetReportParser(0, (HIDReportParser*)&Prs);
  // Serial.println("Ready to scan barcodes...");
}

void loop() { Usb.Task(); }
