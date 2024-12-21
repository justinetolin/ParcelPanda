#include <hidboot.h>
#include <usbhub.h>

// USB Host Shield objects
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);

// Variable to store the barcode data
String barcodeData = "";

// Flag to indicate when a barcode scan is complete
bool barcodeComplete = false;

// Custom Keyboard Parser Class
class CustomKeyboardParser : public HIDReportParser {
 public:
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) override {
    for (uint8_t i = 2; i < len; i++) {
      if (buf[i] > 0) {
        char c = convertKeyCode(buf[i]);
        if (c) {
          if (c == '\n') {
            // End of barcode detected
            barcodeComplete = true;
          } else {
            // Append character to barcode data
            barcodeData += c;
          }
        }
      }
    }
  }

 private:
  char convertKeyCode(uint8_t key) {
    // Map keycodes to characters
    if (key >= 0x04 && key <= 0x1D) {         // Letters (A-Z)
      return 'a' + key - 0x04;                // Convert to lowercase letters
    } else if (key >= 0x1E && key <= 0x27) {  // Numbers (0-9)
      return '0' + key - 0x1E;
    } else if (key == 0x28) {  // Enter key
      return '\n';
    } else if (key == 0x2C) {  // Space
      return ' ';
    }
    return 0;  // Ignore other keys
  }
} customParser;

// Function to read a barcode and return it as a String
String readBarcode() {
  barcodeData = "";         // Clear previous data
  barcodeComplete = false;  // Reset the flag

  while (!barcodeComplete) {
    Usb.Task();  // Process USB tasks
  }

  // Convert to uppercase
  for (size_t i = 0; i < barcodeData.length(); i++) {
    barcodeData[i] = toupper(barcodeData[i]);
  }

  return barcodeData;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing USB Host Shield...");

  // Initialize the USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (1);  // Halt if initialization fails
  }

  Serial.println("USB Host Shield initialized.");

  // Set the custom keyboard parser
  HidKeyboard.SetReportParser(0, &customParser);
}

void loop() {
  Serial.println("Waiting for a barcode scan...");

  // Call the readBarcode() function and print the result
  String scannedBarcode = readBarcode();
  Serial.println("Barcode Data:");
  Serial.println(scannedBarcode);
}
