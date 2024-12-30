#include <SoftwareSerial.h>

// Define pins for GSM module
#define RX_PIN 10
#define TX_PIN 11

// Define LED pin
#define LED_PIN 13

// Create a SoftwareSerial object for GSM module communication
SoftwareSerial gsm(RX_PIN, TX_PIN);

void setup() {
  // Initialize serial communication with the GSM module
  gsm.begin(9600);

  // Initialize LED pin as output
  pinMode(LED_PIN, OUTPUT);

  // Turn off the LED initially
  digitalWrite(LED_PIN, LOW);

  // Allow the GSM module to initialize
  delay(1000);

  // Send initial AT commands to set up SMS functionality
  gsm.println("AT");  // Check communication
  delay(1000);
  gsm.println("AT+CMGF=1");  // Set SMS text mode
  delay(1000);
  gsm.println(
      "AT+CNMI=1,2,0,0,0");  // Configure to show incoming SMS immediately
  delay(1000);
}

void loop() {
  // Check if data is available from GSM module
  if (gsm.available()) {
    String message = gsm.readString();  // Read the incoming SMS

    // Check if the message contains "ON"
    if (message.indexOf("ON") >= 0) {
      digitalWrite(LED_PIN, HIGH);  // Turn on the LED
    }
    // Check if the message contains "OFF"
    else if (message.indexOf("OFF") >= 0) {
      digitalWrite(LED_PIN, LOW);  // Turn off the LED
    }
  }
}
