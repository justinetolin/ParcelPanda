#include <Arduino.h>

// Function declaration
bool checkObject(int sensorPins[2], int& distance1, int& distance2);

void setup() {
  Serial.begin(9600);
  Serial.println(
      "Send '1' for parComp1, '2' for monComp1, '3' for parComp2, '4' for "
      "monComp2 to check for object placement/removal.");
}

void loop() {
  // Check if data is available in the Serial Monitor
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read the command sent via serial

    // Filter out unwanted characters such as newline ('\n') and carriage return
    // ('\r')
    if (command == '\n' || command == '\r') {
      return;  // Ignore newline and carriage return
    }

    int distance1 = 0;
    int distance2 = 0;

    // Define the sensor pins for each sensor
    int parComp[2] = {22, 23};   // Trig and Echo for parComp1
    int monComp[2] = {24, 25};   // Trig and Echo for monComp1
    int scnDect[2] = {26, 27};   // Trig and Echo for parComp2
    int monComp2[2] = {28, 29};  // Trig and Echo for monComp2

    // Use switch case to handle different sensors based on the command
    switch (command) {
      case '1': {
        // Check for object using parComp1
        bool isObjectPresent = checkObject(parComp1, distance1, distance2);
        if (isObjectPresent) {
          Serial.println("parComp1: Object is present.");
          digitalWrite(13, HIGH);  // Turn LED on
        } else {
          Serial.println("parComp1: Object is not present.");
          digitalWrite(13, LOW);  // Turn LED off
        }
        break;
      }
      case '2': {
        // Check for object using monComp1
        bool isObjectPresent = checkObject(monComp1, distance1, distance2);
        if (isObjectPresent) {
          Serial.println("monComp1: Object is present.");
          digitalWrite(13, HIGH);  // Turn LED on
        } else {
          Serial.println("monComp1: Object is not present.");
          digitalWrite(13, LOW);  // Turn LED off
        }
        break;
      }
      case '3': {
        // Check for object using parComp2
        bool isObjectPresent = checkObject(parComp2, distance1, distance2);
        if (isObjectPresent) {
          Serial.println("parComp2: Object is present.");
          digitalWrite(13, HIGH);  // Turn LED on
        } else {
          Serial.println("parComp2: Object is not present.");
          digitalWrite(13, LOW);  // Turn LED off
        }
        break;
      }
      case '4': {
        // Check for object using monComp2
        bool isObjectPresent = checkObject(monComp2, distance1, distance2);
        if (isObjectPresent) {
          Serial.println("monComp2: Object is present.");
          digitalWrite(13, HIGH);  // Turn LED on
        } else {
          Serial.println("monComp2: Object is not present.");
          digitalWrite(13, LOW);  // Turn LED off
        }
        break;
      }
      default:
        Serial.println(
            "Invalid command. Send '1' for parComp1, '2' for monComp1, '3' for "
            "parComp2, '4' for monComp2.");
        break;
    }
  }

  delay(100);  // Small delay to prevent Serial buffer overflow
}

// Function to measure distance using the ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  long totaltime;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  totaltime = pulseIn(echoPin, HIGH);
  int distance = totaltime * 0.034 / 2;  // Calculate distance
  return distance;
}

// Function to check object placement/removal based on sensor input
bool checkObject(int sensorPins[2], int& distance1, int& distance2) {
  int trig = sensorPins[0];
  int echo = sensorPins[1];

  // Set pin modes
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(13, OUTPUT);  // LED pin for indication

  // Measure the initial distance
  distance1 = measureDistance(trig, echo);
  delay(100);  // Delay to allow sensor to stabilize

  // Measure the second distance
  distance2 = measureDistance(trig, echo);

  // Calculate the difference and sum of the distances
  int diffdist = distance1 - distance2;
  int sumDistances = distance1 + distance2;

  // Logic for determining if an object is placed or removed
  if (diffdist > 0 && distance1 > 50 && distance2 < 20) {
    // Object placed (was far, now close)
    return true;
  } else if (diffdist < 0 && distance1 < 20 && distance2 > 50) {
    // Object removed (was close, now far)
    return false;
  } else if (sumDistances < 100) {
    // Object is still there
    return true;
  } else {
    // No object detected
    return false;
  }
}
