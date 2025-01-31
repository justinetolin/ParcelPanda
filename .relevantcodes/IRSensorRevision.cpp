#include <Servo.h>

// Pin assignments
const int photoSensorPin = A0;  // Photosensor connected to analog pin A0
const int ledPin = 13;          // LED connected to pin 13
const int servoPin = 9;         // Servo connected to pin 9

// Servo object
Servo myServo;

// Servo positions
int currentServoPosition = 90;  // Start at 90 degrees
int targetServoPosition = 90;   // Desired position (90 or 180)

void setup() {
  pinMode(ledPin, OUTPUT);              // Set LED pin as output
  myServo.attach(servoPin);             // Attach the servo to the specified pin
  myServo.write(currentServoPosition);  // Initialize servo at 90 degrees
  Serial.begin(9600);                   // Start serial communication (optional)
}

void loop() {
  int lightLevel =
      analogRead(photoSensorPin);  // Read the analog value from the sensor

  // Print the light level for debugging (optional)
  Serial.println(lightLevel);

  // Define a threshold for light detection (adjust as needed)
  int threshold = 500;  // Adjust this value based on your photosensor

  if (lightLevel < threshold) {
    // No light detected
    digitalWrite(ledPin, HIGH);  // Turn ON the LED
    targetServoPosition = 80;    // Set target position to 180 degrees
  } else {
    // Light detected
    digitalWrite(ledPin, LOW);  // Turn OFF the LED
    targetServoPosition = 10;   // Set target position to 90 degrees
  }

  // Move the servo gradually to the target position
  if (currentServoPosition < targetServoPosition) {
    currentServoPosition++;
    myServo.write(currentServoPosition);
    delay(10);  // Slow down the movement
  } else if (currentServoPosition > targetServoPosition) {
    currentServoPosition--;
    myServo.write(currentServoPosition);
    delay(10);  // Slow down the movement
  }
}
