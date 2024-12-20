#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

// ! VARIABLE & PIN DEFINITIONS
#define BUZZER_PIN 13
#define CSpin 53
#define parDoorPin 10
#define monDoorPin 11
#define bluetoothState 44
#define hallSensorPin A0
String User[4] = {"TRACKING", "+639915176440", "false", "123456"};
bool availability = true;
int parDoorAngle = 0;
int monDoorAngle = 0;
const int maxOpenAngle = 180;
const int parCompPins[2] = {22, 23};
const int monCompPins[2] = {24, 25};
const int adminPins[2] = {24, 25};
const int detectionThreshold = 250;
int _timeout;
String _buffer;

// ! MILLIS: TASKS PREVIOUS TIMES & INTERVALS
unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis();
unsigned long prevTime_T4 = millis();
unsigned long prevTime_T5 = millis();

unsigned long interval_T1 = 3000;
unsigned long interval_T2 = 1000;
unsigned long interval_T3 = 1000;
unsigned long interval_T4 = 100;
unsigned long interval_T5 = 1000;

// ! CLASSES DEFINITIONS
File myFile;
Servo parDoorServo;
Servo monDoorServo;
Servo adminDoorServo;

// buzzer tones
const uint16_t success[] PROGMEM = {100, 100, 100, 100};
const uint16_t error[] PROGMEM = {300, 100, 100, 100};
const uint16_t alarm[] PROGMEM = {100, 100, 100, 100, 500};
const uint16_t notif[] PROGMEM = {100};

// ! FUNCTION DEFINITIONS

void playPattern(const uint16_t *patternArray, size_t patternArraySize) {
  for (size_t i = 0; i < patternArraySize; i++) {
    uint16_t duration = pgm_read_word_near(patternArray + i);
    if (i % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    delay(duration);
  }
  digitalWrite(BUZZER_PIN, LOW);
}

void successTone() {
  playPattern(success, sizeof(success) / sizeof(success[0]));
}
void errorTone() { playPattern(error, sizeof(error) / sizeof(error[0])); }
void notifTone() { playPattern(notif, sizeof(notif) / sizeof(notif[0])); }
void alarmTone(bool active) {
  static bool isActive = false;
  if (active) {
    isActive = true;
  } else {
    isActive = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  while (isActive) {
    playPattern(alarm, sizeof(alarm) / sizeof(alarm[0]));
  }
}

void sysLog(const String &logMessage) {
  myFile = SD.open("LOGS.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(logMessage);
    myFile.close();
    Serial.println("Log written to LOGS.txt.");
  } else {
    Serial.println("Error opening LOGS.txt for writing.");
  }
}

bool writeUser(String userData[4]) {
  myFile = SD.open("USERINFO.txt", FILE_WRITE);
  if (myFile) {
    for (int i = 0; i < 4; i++) {
      myFile.print(userData[i]);
      if (i < 3) myFile.print(",");
    }
    myFile.println();
    myFile.close();
    Serial.println("New instance have been saved to SD.");
    return true;
  } else {
    Serial.println("Error opening USERINFO.txt for writing.");
    return false;
  }
}

void readUser(String outputData[4]) {
  myFile = SD.open("USERINFO.txt");
  if (myFile) {
    String fileContent = "";
    while (myFile.available()) {
      char c = myFile.read();
      fileContent += c;
    }
    myFile.close();

    int index = 0;
    int start = 0;
    for (int i = 0; i < fileContent.length(); i++) {
      if (fileContent[i] == ',' || fileContent[i] == '\n') {
        outputData[index++] = fileContent.substring(start, i);
        start = i + 1;
        if (index >= 4) break;
      }
    }
    parDoor(true);
    Serial.println("Data read successfully.");
  } else {
    Serial.println("Error opening USERINFO.txt for reading.");
  }
}

bool availabilityCheck() {
  if (!SD.exists("USERINFO.txt")) {
    availability = true;
  } else {
    availability = false;
  }
  return availability;
}

String NewInstance() {
  Serial1.println("1");
  String tempUser[3];
  int index = 0;
  String receivedData = "";

  Serial.println("Waiting indefinitely for user data...");

  unsigned long startTime = millis();
  while (millis() - startTime < 300000) {
    if (Serial1.available()) {
      receivedData = Serial1.readStringUntil('\n');
      receivedData.trim();
      break;
    }
  }

  if (receivedData == "") {
    Serial.println(
        "Timeout reached: No data received for the first three values.");
    return "000000";
  }

  int start = 0;

  for (int i = 0; i < receivedData.length(); i++) {
    if (receivedData[i] == ',' || receivedData[i] == '\n') {
      tempUser[index++] = receivedData.substring(start, i);
      start = i + 1;
      if (index >= 3) break;
    }
  }

  for (int i = 0; i < 3; i++) {
    User[i] = tempUser[i];
  }

  Serial.println("First three values received and stored.");
  Serial.println("Waiting indefinitely for PIN...");

  String pinData = "";

  startTime = millis();
  while (millis() - startTime < 300000) {
    if (Serial1.available()) {
      pinData = Serial1.readStringUntil('\n');
      pinData.trim();
      break;
    }
  }

  if (pinData.length() == 6) {
    User[3] = pinData;

    if (writeUser(User)) {
      readUser(User);
      bool option = (User[2] == "true");  // Determine if the option is true
      if (option) {
        return User[3];  // Corrected from `readData[3]`
      } else {
        monDoor(true);
        while (!isObjectPresent(monCompPins)) {
          delay(500);
        }
        delay(3000);
        monDoor(false);
        return User[3];  // Corrected from `readData[3]`
      }
    } else {
      return "000000";
    }
  } else {
    Serial.println("Timeout reached: PIN not received in time or invalid PIN.");
    return "000000";
  }
}

void clearUser() {
  // Open the original file for reading
  File readFile = SD.open("USERINFO.txt", FILE_READ);
  if (readFile) {
    File prevUsersFile = SD.open("USERLOG.txt", FILE_WRITE);
    if (prevUsersFile) {
      while (readFile.available()) {
        char c = readFile.read();
        prevUsersFile.print(c);
      }

      readFile.close();
      prevUsersFile.close();

      SD.remove("USERINFO.txt");

      Serial.println("User data moved to USERLOG.txt");
      parDoor(false);
    } else {
      Serial.println("Error opening PREVUSERS.txt");
      readFile.close();
    }
  } else {
    Serial.println("Error opening USERINFO.txt");
  }
}

void parDoor(bool open) {
  parDoorServo.attach(parDoorPin);

  if (open) {
    parDoorAngle = maxOpenAngle;
    Serial.println("Opening Parent Door...");
  } else {
    parDoorAngle = 0;
    Serial.println("Closing Parent Door...");
  }

  for (int pos = parDoorServo.read(); pos != parDoorAngle;
       pos += (parDoorAngle > pos ? 1 : -1)) {
    parDoorServo.write(pos);
    delay(15);
  }
  parDoorServo.detach();
}

void monDoor(bool open) {
  monDoorServo.attach(monDoorPin);

  if (open) {
    monDoorAngle = maxOpenAngle;
    Serial.println("Opening Monitor Door...");
  } else {
    monDoorAngle = 0;
    Serial.println("Closing Monitor Door...");
  }

  // Smoothly move the servo to the target angle
  for (int pos = monDoorServo.read(); pos != monDoorAngle;
       pos += (monDoorAngle > pos ? 1 : -1)) {
    monDoorServo.write(pos);
    delay(15);  // Adjust speed of movement
  }

  monDoorServo.detach();
}

void BoxSetup() {
  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    receivedData.trim();
    Serial.println("RECEIVED: ");
    Serial.println(receivedData);

    if (receivedData == "check") {
      Serial1.println(availabilityCheck());
      Serial.println(availabilityCheck());
      notifTone();
    } else if (receivedData == "NewInstance") {
      String pinResponse = NewInstance();
      Serial1.println(pinResponse);
      notifTone();
    } else if (receivedData == "Clear") {
      clearUser();
      Serial1.println("Clear Success!");
    }
  }
}

int measureDistance(int trigPin, int echoPin) {
  long totaltime;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  totaltime = pulseIn(echoPin, HIGH);
  int distance = totaltime * 0.034 / 2;
  return distance;
}

bool isObjectPresent(int sensorPins[2]) {
  int trig = sensorPins[0];
  int echo = sensorPins[1];
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  int distance1 = measureDistance(trig, echo);
  delay(100);
  int distance2 = measureDistance(trig, echo);

  int diffdist = distance1 - distance2;
  int sumdist = distance1 + distance2;

  if (diffdist > 0 && distance1 > 50 && distance2 < 20) {
    return true;  // Object placed (was far, now close)
  } else if (diffdist < 0 && distance1, 20 && distance2 > 50) {
    return false;  // Object removed (was close, now far)
  } else if (sumdist < 100) {
    return true;  // Object still present
  } else {
    return false;  // Object is absent
  }
}

void sendText(String number, String message) {
  Serial2.println("AT+CMGF=1");
  delay(200);
  Serial2.println("AT+CMGS=\"" + number + "\"\r");
  delay(200);
  Serial2.println(message);
  delay(200);
  Serial2.println((char)26);
  delay(200);
  _buffer = _readSerial();
  Serial.println("Message sent!");
}

String receiveText() {
  Serial2.println("AT+CMGF=1");
  delay(200);
  Serial2.println("AT+CNMI=1,2,0,0,0");
  delay(200);

  _buffer = _readSerial();
  if (_buffer.indexOf("+CMT:") != -1) {
    int msgStartIndex = _buffer.indexOf("\n", _buffer.indexOf("+CMT:")) + 1;
    int msgEndIndex = _buffer.indexOf("\r", msgStartIndex);
    if (msgStartIndex != -1 && msgEndIndex != -1) {
      return _buffer.substring(msgStartIndex, msgEndIndex);
    }
  }
  return "";
}

void callNumber(String number) {
  Serial2.print("ATD");
  Serial2.print(number);
  Serial2.print(";\r\n");
  _buffer = _readSerial();
  Serial.println("Calling " + number);
}

String _readSerial() {
  _timeout = 0;
  while (!Serial2.available() && _timeout < 12000) {
    delay(13);
    _timeout++;
  }
  if (Serial2.available()) {
    return Serial2.readString();
  }
  return "";
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CSpin, OUTPUT);
  pinMode(bluetoothState, INPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  randomSeed(analogRead(10));
  _buffer.reserve(50);

  if (!SD.begin(CSpin)) {
    Serial.println("SD card initialization failed.");
    while (true);
  }
  Serial.println("SD card is ready to use.");

  parDoorServo.attach(parDoorPin);
  parDoorServo.write(parDoorAngle);
  parDoorServo.detach();

  monDoorServo.attach(monDoorPin);
  monDoorServo.write(monDoorAngle);
  monDoorServo.detach();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime_T1 >= interval_T1) {
    prevTime_T1 = currentTime;
    BoxSetup();
  }

  if (currentTime - prevTime_T2 >= interval_T2) {
    prevTime_T2 = currentTime;
    int effectValue = analogRead(hallSensorPin);
    if (effectValue > detectionThreshold) {
      // no magnetic field detected ie STOLEN
      // run a function with while loop waiting for user to respond to the
      // alarm until the beep stops
      alarmTone(true);

      // wait for user response in message
      String message1 = "Your Parcel has been stolen. Alarm is turned on.";
      String message2 =
          "Please reply 'STOP' to acknowledge the theft and stop the alarm. "
          "You can manually input your pin in the keypad as an alternative "
          "action.";
      sendText(User[2], message1);
      delay(200);
      sendText(User[2], message2);
      callNumber(User[2]);
      delay(7000);

      while (true) {
        _buffer = receiveText();
        if (_buffer.equalsIgnoreCase("STOP")) {
          Serial.println("User acknowledge theft. Stopping alarm...");
          sendText(User[2], "Acknowledged. Stopping alarm now.");
          clearUser();
          break;
        }
      }

      // wait for code in keypad
    } else {
      alarmTone(false);
    }
  }

  if (currentTime - prevTime_T3 >= interval_T3) {
    prevTime_T3 = currentTime;
    bool parComp = isObjectPresent(parCompPins);
    if (!availability && parComp) {
      Serial.println("PARCEL INTACT");
    } else if (availability && !parComp) {
      Serial.println("PARCEL INTACT");
    } else if (!availability && !parComp) {
      Serial.println("PARCEL STOLEN");
      // Alarm and notif things
      alarmTone(true);
    }
  }

  // if (currentTime - prevTime_T4 >= interval_T4) {
  //   prevTime_T4 = currentTime;
  //   parseKeypad();
  // }

  // FOR GSM DEBUGGING
  // if (Serial.available() > 0) {
  //   char command = Serial.read();
  //   if (command == 's') {
  //     String number = "+639524882324";  // Replace with desired number
  //     String message = "Hello, this is a test message!";
  //     sendText(number, message);
  //   } else if (command == 'r') {
  //     receiveText();
  //   } else if (command == 'c') {
  //     String number = "+639524882324";  // Replace with desired number
  //     callNumber(number);
  //   }
  // }

  // // Check if there's incoming data from the GSM module
  // if (Serial2.available() > 0) {
  //   String reply = _readSerial();
  //   if (reply.indexOf("ON") != -1) {
  //     digitalWrite(BUZZER_PIN, HIGH);  // Turn on LED if "ON" is received
  //     Serial.println("LED ON");
  //   } else if (reply.indexOf("OFF") != -1) {
  //     digitalWrite(BUZZER_PIN, LOW);  // Turn off LED if "OFF" is received
  //     Serial.println("LED OFF");
  //   }
  // }
}
