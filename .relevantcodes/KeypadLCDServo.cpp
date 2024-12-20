#include <Servo.h>

// Create servo objects
Servo parDoorServo;
Servo monDoorServo;

// Track servo states
bool isParDoorOpen = false;  // State for servo on pin 10
bool isMonDoorOpen = false;  // State for servo on pin 11

// Function to move servo to a specific angle with duration
void moveServo(int servoPin, int angle, int duration) {
  Servo *currentServo;
  if (servoPin == 10) {
    currentServo = &parDoorServo;
  } else if (servoPin == 11) {
    currentServo = &monDoorServo;
  } else {
    Serial.println("Invalid servo pin");
    return;
  }

  currentServo->attach(servoPin);

  // Ensure the angle is within the servo's range
  if (angle > 180) angle = 180;
  if (angle < 0) angle = 0;

  // Move servo to specified angle
  currentServo->write(angle);
  delay(duration);

  // Detach servo to prevent jitter
  currentServo->detach();

  Serial.print("Servo on pin ");
  Serial.print(servoPin);
  Serial.print(" moved to angle ");
  Serial.println(angle);
}

// Toggle function to open/close a door
void toggleDoor(int doorPin, bool &isDoorOpen) {
  if (isDoorOpen) {
    moveServo(doorPin, 0, 1000);  // Close the door
    Serial.print("Door on pin ");
    Serial.print(doorPin);
    Serial.println(" closed.");
  } else {
    moveServo(doorPin, 180, 1000);  // Open the door
    Serial.print("Door on pin ");
    Serial.print(doorPin);
    Serial.println(" opened.");
  }
  isDoorOpen = !isDoorOpen;  // Toggle state
}

void setup() {
  Serial.begin(9600);
  Serial.println("Servo Control Ready");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case '1':  // Toggle parent door (pin 10)
        toggleDoor(10, isParDoorOpen);
        break;

      case '2':  // Toggle monitor door (pin 11)
        toggleDoor(11, isMonDoorOpen);
        break;

      default:
        Serial.println("Invalid command");
    }
  }
}
