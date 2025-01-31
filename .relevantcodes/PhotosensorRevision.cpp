#include <Servo.h>

// Define the pins for the sensors and servo
const int photoSensorPin = A0;  // Photosensor connected to A0
const int servoPin = 7;         // Servo connected to pin 7

// Define thresholds
const int photoThreshold =
    500;  // Adjust this value based on the photosensor's response

Servo myServo;  // Create a Servo object

void setup() {
  myServo.attach(servoPin);  // Attach the servo to pin 7
  myServo.write(0);          // Start the servo at 0 degrees
  Serial.begin(9600);        // Initialize serial communication
}

void loop() {
  // Read sensor values
  int photoValue = analogRead(photoSensorPin);

  // Check the photosensor condition (covered/no light)
  if (photoValue < photoThreshold) {
    Serial.println("Photosensor covered: Servo moving to 180");

    // Move the servo from 0 to 180 degrees slowly
    for (int pos = 0; pos <= 180; pos++) {
      myServo.write(pos);
      delay(15);  // Adjust for slower movement
    }

    delay(2000);  // Wait for 2 seconds

    // Move the servo back to 0 degrees slowly
    for (int pos = 180; pos >= 0; pos--) {
      myServo.write(pos);
      delay(15);  // Adjust for slower movement
    }
  }

  delay(100);  // Small delay for stability
}
