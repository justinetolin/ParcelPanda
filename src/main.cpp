#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <hidboot.h>
#include <usbhub.h>

// ! VARIABLE & PIN DEFINITIONS
#define BUZZER_PIN 3
#define CSpin 4
#define parDoorPin 10
#define monDoorPin 11
#define bluetoothState 44
#define hallSensorPin A0
String User[4] = {"TRACKING", "+639915176440", "false", "123456"};
bool availability = true;
bool received = false;
int parDoorAngle = 0;
int monDoorAngle = 0;
const int maxOpenAngle = 180;
const int parCompPins[2] = {22, 23};
const int monCompPins[2] = {24, 25};
const int adminPins[2] = {24, 25};
const int detectionThreshold = 250;
int _timeout;
String _buffer;
String barcodeData = "";
bool barcodeComplete = false;
enum State { SETUP, DELIVERY, RETRIEVAL };
State currentState = SETUP;

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

// ! CLASSES/OBJECTS DEFINITIONS
File myFile;
Servo parDoorServo;
Servo monDoorServo;
Servo adminDoorServo;
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> HidKeyboard(&Usb);

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

// buzzer tones
const uint16_t success[] PROGMEM = {100, 100, 100, 100};
const uint16_t error[] PROGMEM = {300, 100, 100, 100};

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
void alarmTone(bool active) { digitalWrite(BUZZER_PIN, active ? HIGH : LOW); }

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

String readBarcode() {
  if (!barcodeComplete) {
    return "";
  }

  String result = barcodeData;
  barcodeData = "";
  barcodeComplete = false;

  // Convert to uppercase
  for (size_t i = 0; i < barcodeData.length(); i++) {
    barcodeData[i] = toupper(barcodeData[i]);
  }

  return result;
}

void SecurityOne() {
  String message1 = "Your Parcel has been stolen. Alarm is turned on.";
  String message2 =
      "Please reply 'STOP' to acknowledge the theft and stop the alarm.";
  String message3 =
      "You can manually input your pin in the keypad as an alternative "
      "action.";
  Serial.println("Sending theft notification...");
  sendText(User[2], message1);
  delay(1000);
  sendText(User[2], message2);
  delay(1000);
  sendText(User[2], message3);
  delay(1000);
  callNumber(User[2]);
  delay(7000);

  unsigned long waitStartTime = millis();
  while (millis() - waitStartTime < 60000) {
    _buffer = receiveText();
    if (_buffer.equalsIgnoreCase("STOP")) {
      Serial.println("User acknowledge theft. Stopping alarm...");
      sendText(User[2], "Acknowledged. Stopping alarm now.");
      clearUser();
      break;
    }
    delay(100);
  }
  alarmTone(false);
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
  SPI.begin();

  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (1);  // Halt if initialization fails
  }
  Serial.println("USB Host Shield initialized.");

  if (!SD.begin(CSpin)) {
    Serial.println("SD card initialization failed.");
    while (true);
  }
  Serial.println("SD card is ready to use.");

  HidKeyboard.SetReportParser(0, &customParser);

  parDoorServo.attach(parDoorPin);
  parDoorServo.write(parDoorAngle);
  parDoorServo.detach();

  monDoorServo.attach(monDoorPin);
  monDoorServo.write(monDoorAngle);
  monDoorServo.detach();
}

void loop() {
  unsigned long currentTime = millis();

  switch (currentState) {
    case SETUP:
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
          received = false;
        } else if (receivedData == "Clear") {
          clearUser();
          Serial1.println("Clear Success!");
        }
      }
      SecurityOne();
      currentState = DELIVERY;
      break;

    case DELIVERY:
      if (currentTime - prevTime_T1 >= interval_T1) {
        prevTime_T1 = currentTime;
        String scannedBarcode = readBarcode();
        if (!received) {
          Serial.println("Barcode Data:");
          Serial.println(scannedBarcode);

          if (User[0] == scannedBarcode) {
            parDoor(true);
            while (!isObjectPresent(parCompPins)) {
              delay(500);
            }
            Serial.println("Parcel placed");
            int countdown = 5;  // Countdown time in seconds
            while (countdown > 0) {
              Serial.print("Door will close in: ");
              Serial.println(countdown);
              delay(1000);  // Wait for 1 second before reducing the countdown
              countdown--;  // Decrease countdown by 1
            }
            parDoor(false);

            if (User[2] == "false") {
              monDoor(true);
              while (!isObjectPresent(monCompPins)) {
                delay(500);
              }
            }
            Serial.println("Payment retrieved.");
            countdown = 2;  // Countdown time in seconds
            while (countdown > 0) {
              Serial.print("Door will close in: ");
              Serial.println(countdown);
              delay(1000);  // Wait for 1 second before reducing the countdown
              countdown--;  // Decrease countdown by 1
            }
            monDoor(false);
          }
        }
      }
      break;
    case RETRIEVAL:
      break;
  }

  if (currentTime - prevTime_T2 >= interval_T2) {
    prevTime_T2 = currentTime;
    int effectValue = analogRead(hallSensorPin);
    if (effectValue > detectionThreshold) {
      // no magnetic field detected ie STOLEN
      // run a function with while loop waiting for user to respond to the
      // alarm until the beep stops
      alarmTone(true);
      SecurityOne();
      // wait for code in keypad
    } else if (effectValue > detectionThreshold) {
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
