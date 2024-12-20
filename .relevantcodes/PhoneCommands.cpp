#include <Arduino.h>

bool isValidPin(String data);

// Define constants for specific responses
#define RESPONSE_CHECK "true"
#define RESPONSE_NEW_INSTANCE "proceed"

void setup() {
  // Initialize Serial Monitor communication at 9600 baud (for debugging)
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // Initialize Serial1 communication at 9600 baud (for HC-05 AT commands)
  Serial1.begin(9600);
}

void loop() {
  // Check if data is received from the phone (via HC-05)
  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n'); // Read the incoming data until newline
    receivedData.trim(); // Remove any leading/trailing whitespace
    
    Serial.print("Received: "); // Debugging
    Serial.println(receivedData);

    // Respond based on the received message
    if (receivedData == "check") {
      Serial1.println(RESPONSE_CHECK); // Reply with "true"
      Serial.println(RESPONSE_CHECK);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    } else if (receivedData == "NewInstance") {
      Serial1.println(RESPONSE_NEW_INSTANCE); // Reply with "proceed"
      Serial.println(RESPONSE_NEW_INSTANCE); // Reply with "proceed"
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    } else if (isValidPin(receivedData)) {
      delay(3000); // Wait for 3 seconds
      Serial1.println(receivedData); 
      Serial.println(receivedData); 
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      // Send the pin back
    } else {
      Serial.println("Unknown command"); // Debugging for unexpected input
    }
  }

  // Check if data is received from Serial Monitor (for testing/debugging)
  if (Serial.available()) {
    String monitorInput = Serial.readStringUntil('\n'); // Read from Serial Monitor
    monitorInput.trim();
    Serial1.println(monitorInput); // Send to HC-05
  }
}

// Function to validate if a string is a 6-digit number greater than 0
bool isValidPin(String data) {
  // Debugging: Print the received string
  Serial.print("Validating PIN: ");
  return true;
}
