#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Password.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <hidboot.h>
#include <usbhub.h>

// ! VARIABLE & PIN DEFINITIONS
#define BUZZER_PIN 3
#define CSpin 4
#define parDoorPin 6
#define monDoorPin 7
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

String newPasswordString;
char newPassword[7];
byte a = 5;
bool value = true;
bool isChangingPassword = false;
bool isVerifyingOldPassword = false;
Password password = Password(User[3]);
byte maxPasswordLength = 6;
byte currentPasswordLength = 0;
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

// ! MILLIS: TASKS PREVIOUS TIMES & INTERVALS
unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis();
unsigned long prevTime_T4 = millis();
unsigned long prevTime_T5 = millis();

unsigned long interval_T1 = 3000;
unsigned long interval_T2 = 200;
unsigned long interval_T3 = 1000;
unsigned long interval_T4 = 100;
unsigned long interval_T5 = 1000;

// ! CLASSES/OBJECTS DEFINITIONS
File myFile;
Servo parDoorServo;
Servo monDoorServo;
Servo adminDoorServo;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
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
        // currentState = DELIVERY; // OVERRIDE FOR BUILDING
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
  currentState = SETUP;
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
  barcodeComplete = false;  // Ensure it's reset for the next read
  return barcodeData;
}

void SecurityOne() {
  int effectValue = analogRead(hallSensorPin);
  if (effectValue > detectionThreshold) {
    // no magnetic field detected ie STOLEN
    // run a function with while loop waiting for user to respond to the
    // alarm until the beep stops
    alarmTone(true);
    String message1 = "Your Parcel has been stolen. Alarm is turned on.";
    String message2 =
        "Please reply 'STOP' to acknowledge the theft and stop the alarm.";
    String message3 =
        "You can manually input your pin in the keypad as an alternative "
        "action.";
    Serial.println("Sending theft notification...");
    sendText(User[1], message1);
    delay(1000);
    sendText(User[1], message2);
    delay(1000);
    sendText(User[1], message3);
    delay(1000);
    callNumber(User[1]);
    delay(7000);

    unsigned long waitStartTime = millis();
    while (millis() - waitStartTime < 60000) {
      _buffer = receiveText();
      if (_buffer.equalsIgnoreCase("STOP")) {
        // User acknowledged the theft and wants to stop the alarm
        Serial.println("User acknowledge theft. Stopping alarm...");
        sendText(User[2], "Acknowledged. Stopping alarm now.");
        clearUser();
        alarmTone(false);
        break;
      }

      // Also listen for PIN input on the keypad during alarm
      if (keypad.getKey()) {
        char pressedKey = keypad.getKey();
        static String enteredPin = "";  // Buffer for PIN entry

        if (isdigit(pressedKey)) {
          enteredPin += pressedKey;
        }

        // If the PIN is 6 digits long, check if it matches
        if (enteredPin.length() == 6) {
          if (enteredPin == User[3]) {
            // Stop the alarm and reset the system if PIN matches
            Serial.println("PIN entered, stopping alarm...");
            sendText(User[2], "PIN entered. Stopping alarm now.");
            clearUser();
            alarmTone(false);
            enteredPin = "";  // Reset PIN buffer
            break;
          } else {
            lcd.clear();
            lcd.setCursor(0, 3);
            lcd.print("Incorrect PIN.");
            enteredPin = "";  // Reset PIN buffer
            errorTone();
            delay(2000);
            lcd.clear();
          }
        }
      }
      delay(100);  // Small delay to prevent excessive CPU usage
    }
  } else {
    alarmTone(false);
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CSpin, OUTPUT);
  pinMode(bluetoothState, INPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("<INITIALIZING>");
  delay(3000);
  lcd.clear();

  randomSeed(analogRead(10));
  _buffer.reserve(50);
  SPI.begin();

  lcd.setCursor(0, 1);
  lcd.print("SD Card...");
  if (!SD.begin(CSpin)) {
    Serial.println("SD card initialization failed.");
    while (true);
  }
  Serial.println("SD card is ready to use.");
  delay(500);
  lcd.setCursor(17, 1);
  lcd.print("OK");

  lcd.setCursor(0, 2);
  lcd.print("Scanner...");
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (1);
  }
  Serial.println("USB Host Shield initialized.");
  delay(500);
  lcd.setCursor(17, 2);
  lcd.print("OK");

  HidKeyboard.SetReportParser(0, &customParser);

  parDoorServo.attach(parDoorPin);
  parDoorServo.write(parDoorAngle);
  parDoorServo.detach();

  monDoorServo.attach(monDoorPin);
  monDoorServo.write(monDoorAngle);
  monDoorServo.detach();

  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Welcome to");
  lcd.setCursor(4, 1);
  lcd.print("PARCEL PANDA");
  lcd.setCursor(2, 3);
  lcd.print("Connect to setup;")
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime_T1 >= interval_T1) {
    prevTime_T1 = currentTime;

    SecurityOne();
    // wait for code in keypad
  }

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
      currentState = DELIVERY;
      break;

    case DELIVERY:
      // if (currentTime - prevTime_T2 >= interval_T2) {
      //   prevTime_T2 = currentTime;
      String scannedBarcode = readBarcode();
      if (!received) {
        String scannedBarcode = readBarcode();
        if (scannedBarcode != "") {
          Serial.println("Barcode Scanned: " + scannedBarcode);
        } else {
          Serial.println("No barcode data available yet.");
        }

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
      // }
      break;
    case RETRIEVAL:
      // Step 1: Handle PIN Entry
      if (keypad.getKey()) {
        char pressedKey = keypad.getKey();
        static String enteredPin = "";  // Buffer for PIN entry

        // Check if the entered key is a digit
        if (isdigit(pressedKey)) {
          enteredPin += pressedKey;
          lcd.setCursor(0, 3);
          lcd.print("PIN: ");
          lcd.print(enteredPin);  // Show entered PIN on the LCD
        }

        // If the PIN is 6 digits long, validate it
        if (enteredPin.length() == 6) {
          if (enteredPin == User[3]) {
            // PIN matches, open parent door and wait for parcel removal
            parDoor(true);
            while (isObjectPresent(parCompPins)) {
              delay(500);  // Keep checking if the parcel is still present
            }
            // Parcel has been removed, close the door
            parDoor(false);
            enteredPin = "";  // Reset PIN buffer
            Serial.println("Parcel retrieved successfully.");
          } else {
            lcd.clear();
            lcd.setCursor(0, 3);
            lcd.print("Incorrect PIN.");
            enteredPin = "";  // Reset PIN buffer
            errorTone();      // Play error tone
            delay(2000);      // Wait for a moment before clearing message
            lcd.clear();
          }
        }
      }

      // Step 2: Active Anti-Theft Check
      if (!isObjectPresent(parCompPins)) {
        // If parcel is absent, invoke theft security function
        SecurityTwo();
      }
      break;
  }
}
