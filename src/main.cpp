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
#define CSpin A15
#define parDoorPin 6
#define monDoorPin 7
#define bluetoothState 44
#define hallSensorPin A0
#define resetFunc() asm volatile("jmp 0")

String User[4] = {"TRACKING", "+639915176440", "false", "123456"};

bool availability = true;
bool received = false;
bool SecLVL1 = false;
bool alerted = false;
int effectValue;

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

enum State { SETUP = 0, DELIVERY = 1, RETRIEVAL = 2 };
State currentState = SETUP;
bool isRetrieval = false;

String newPasswordString;
char newPassword[7];
byte a = 5;
bool value = true;
Password password = Password("123456");
byte maxPasswordLength = 6;
byte currentPasswordLength = 0;
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {30, 31, 32, 33};
byte colPins[COLS] = {34, 35, 36, 37};

const byte bell[8] = {0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x00, 0x04, 0x00};

// ! MILLIS: TASKS PREVIOUS TIMES & INTERVALS
unsigned long currentTime;
unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis();
unsigned long prevTime_T4 = millis();
unsigned long prevTime_T5 = millis();

unsigned long interval_T1 = 3000;
unsigned long interval_T2 = 2000;
unsigned long interval_T3 = 100;
unsigned long interval_T4 = 100;
unsigned long interval_T5 = 30000;

int notificationStep = 0;
bool notificationInProgress = false;

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
    Serial.println("Opening Parcel Door...");
  } else {
    parDoorAngle = 0;
    Serial.println("Closing Parcel Door...");
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
    Serial.println("Opening Money Door...");
  } else {
    monDoorAngle = 0;
    Serial.println("Closing Money Door...");
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

void receiveData(String &formData) {
  unsigned long startTime = millis();
  while (millis() - startTime < 300000) {  // 5-minute timeout
    if (Serial1.available()) {
      formData = Serial1.readStringUntil('\n');
      formData.trim();
      return;
    }
  }
  formData = "";  // Default to empty string if timeout occurs
}

String NewInstance() {
  Serial1.println("1");  // Notify Bluetooth module
  Serial.println("Waiting for TRACKING ID...");
  receiveData(User[0]);

  Serial.println("Waiting for phone number...");
  receiveData(User[1]);

  Serial.println("Waiting for option (true/false)...");
  receiveData(User[2]);

  Serial.println("Waiting for PIN...");
  receiveData(User[3]);

  if (writeUser(User)) {
    Serial.println("Data successfully written to SD card.");

    readUser(User);                     // Verify saved data
    bool option = (User[2] == "true");  // Determine if the option is true
    if (option) {
      return User[3];
    } else {
      monDoor(true);
      while (!isObjectPresent(monCompPins)) {
        delay(500);
      }
      delay(3000);
      monDoor(false);

      newPasswordString = User[3];
      newPasswordString.toCharArray(
          newPassword, maxPasswordLength + 1);  // Convert string to char array
      password.set(newPassword);
      currentState = DELIVERY;
      Serial.println("State Switch: DELIVERY");
      return User[3];
    }
  } else {
    Serial.println("Error writing data to SD card.");
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
  Serial.println("State Switch: SETUP");
}

// String _readSerial() {
//   _timeout = 0;
//   while (!Serial2.available() && _timeout < 5000) {
//     delay(13);
//     _timeout++;
//   }
//   if (Serial2.available()) {
//     return Serial2.readString();
//   }
//   return "";
// }

String _readSerial() {
  unsigned long startMillis = millis();
  // while (!Serial2.available() && (millis() - startMillis) < 5000) {
  //   Serial.println("readSerial returned NOTHING.");
  // }
  if (Serial2.available()) {
    Serial.println("readSerial returned DATA.");
    return Serial2.readString();
  }
  Serial.println("readSerial returned NOTHING.");
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

void processNumberKey(char key) {
  lcd.setCursor(a, 1);
  lcd.print("*");
  a++;
  if (a == 11) {
    a = 5;
  }
  currentPasswordLength++;
  password.append(key);

  if (currentPasswordLength == maxPasswordLength) {
    evaluatePassword();
  }
}

void evaluatePassword() {
  if (password.evaluate()) {
    digitalWrite(BUZZER_PIN, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Parcel Comp");
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
    lcd.setCursor(17, 0);
    lcd.print("OK");
    parDoor(true);
    lcd.setCursor(0, 2);
    lcd.print("Please get your");
    lcd.setCursor(0, 3);
    lcd.print("parcel.");
    while (1) {
      if (!isObjectPresent(parCompPins)) {
        delay(1000);
        parDoor(false);
        delay(500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Thanks for using");
        lcd.setCursor(0, 1);
        lcd.print("PARCEL PANDA!");
        delay(2000);
        lcd.setCursor(0, 3);
        lcd.print("Clearing user...");
        delay(500);
        clearUser();
        break;
      }
    }
  } else {
    showError();  // Show an error if the password is incorrect
  }
  resetPassword();  // Reset the password for the next attempt
}

void showError() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  lcd.setCursor(0, 0);
  lcd.print("WRONG PASSWORD!");
  lcd.setCursor(0, 1);
  lcd.print("PLEASE TRY AGAIN");
  delay(2000);
  lcd.clear();
  a = 5;
}

void resetPassword() {
  password.reset();
  currentPasswordLength = 0;
  lcd.clear();
  a = 5;
}

void SecurityOne() {
  lcd.setCursor(0, 3);
  lcd.write(byte(0));
  lcd.setCursor(2, 3);
  lcd.print("SECURITY LEVEL 1");
  lcd.setCursor(19, 3);
  lcd.write(byte(0));

  effectValue = analogRead(hallSensorPin);
  if (effectValue > detectionThreshold && !alerted) {
    lcd.setCursor(0, 2);
    lcd.print("Notifying user...");
    // no magnetic field detected ie STOLEN
    // run a function with while loop waiting for user to respond to the
    // alarm until the beep stops
    alarmTone(true);
    String message1 = "Entire box has been stolen. Alarm is turned on.";
    String message2 =
        "Please reply 'STOP' to acknowledge the theft and stop the alarm.";
    String message3 =
        "You can manually input your pin in the keypad as an alternative.";
    Serial.println("Sending theft notification...");
    if (currentTime - prevTime_T2 >= interval_T2) {
      prevTime_T2 = currentTime;
      sendText(User[1], message1);
      Serial.println("Message 1 sent.");
      delay(2000);
      sendText(User[1], message2);
      Serial.println("Message 2 sent.");
      delay(2000);
      sendText(User[1], message3);
      Serial.println("Message 3 sent.");
      delay(2000);
      // callNumber(User[1]);
      Serial.println("Calling user...");
      delay(7000);
      alerted = true;
      lcd.setCursor(0, 2);
      lcd.print("   User notified");
    }
  }
}

void SecurityTwo() {
  Serial.println("Security Two executed successfully.");
  lcd.setCursor(0, 3);
  lcd.write(byte(0));
  lcd.setCursor(2, 3);
  lcd.print("SECURITY LEVEL 2");
  lcd.setCursor(19, 3);
  lcd.write(byte(0));

  if (!alerted) {
    lcd.setCursor(0, 2);
    lcd.print("Notifying user...");
    // no magnetic field detected ie STOLEN
    // run a function with while loop waiting for user to respond to the
    // alarm until the beep stops
    alarmTone(true);
    String message1 = "Your Parcel has been stolen. Alarm is turned on.";
    String message2 =
        "Please reply 'STOP' to acknowledge the theft and stop the alarm.";
    String message3 =
        "You can manually input your pin in the keypad as an alternative.";
    Serial.println("Sending theft notification...");
    if (currentTime - prevTime_T2 >= interval_T2) {
      prevTime_T2 = currentTime;
      sendText(User[1], message1);
      Serial.println("Message 1 sent.");
      delay(2000);
      sendText(User[1], message2);
      Serial.println("Message 2 sent.");
      delay(2000);
      sendText(User[1], message3);
      Serial.println("Message 3 sent.");
      delay(2000);
      alerted = true;
      lcd.setCursor(0, 2);
      lcd.print("   User notified");
    }
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
  lcd.setCursor(3, 0);
  lcd.print("<INITIALIZING>");
  lcd.createChar(0, bell);
  delay(1000);

  randomSeed(analogRead(10));
  _buffer.reserve(50);
  SPI.begin();

  lcd.setCursor(0, 1);
  lcd.print("SD Card...");
  if (!SD.begin(CSpin)) {
    Serial.println("SD card initialization failed.");
    while (true) {
      lcd.setCursor(17, 1);
      lcd.print("X");
      delay(500);
      lcd.setCursor(17, 1);
      lcd.print(" ");
    };
  }
  Serial.println("SD card is ready to use.");
  delay(500);
  lcd.setCursor(17, 1);
  lcd.print("OK");

  lcd.setCursor(0, 2);
  lcd.print("Scanner...");
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (true) {
      lcd.setCursor(17, 1);
      lcd.print("X");
      delay(500);
      lcd.setCursor(17, 1);
      lcd.print(" ");
    };
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

  delay(3000);
  lcd.clear();
  lcd.setCursor(5, 0);
  lcd.print("Welcome to");
  lcd.setCursor(4, 1);
  lcd.print("PARCEL PANDA");
  lcd.setCursor(2, 3);
  lcd.print("Connect to setup;");

  if (!availabilityCheck() && !received) {
    currentState = DELIVERY;
    Serial.println("State Switch: DELIVERY");
    Serial.println("Existing user detected.");
    readUser(User);

    newPasswordString = User[3];
    newPasswordString.toCharArray(
        newPassword, maxPasswordLength + 1);  // Convert string to char array
    password.set(newPassword);
    newPasswordString = "";
  } else if (availabilityCheck() && !received) {
    currentState = SETUP;
    Serial.println("State Switch: SETUP");
    Serial.println("No existing user detected.");
  } else if (!availabilityCheck() && received) {
    currentState = RETRIEVAL;
    isRetrieval = true;
    Serial.println("State Switch: RETRIEVAL");
    Serial.println("Panda has received the parcel. Await retrieval.");
  }
}

void loop() {
  Serial.print("Loop running... STATE: ");
  Serial.println(currentState);

  currentTime = millis();

  if (currentTime - prevTime_T1 >= interval_T1) {
    prevTime_T1 = currentTime;
    effectValue = analogRead(hallSensorPin);
    if (effectValue > detectionThreshold) {
      lcd.clear();
      Serial.println("SECURITY LEVEL 1 TRIGGERED");
      while (1) {
        SecurityOne();
        if (Serial2.available()) {
          String message = Serial2.readString();

          if (message.indexOf("STOP") >= 0) {
            // User acknowledged the theft and wants to stop the alarma
            Serial.println("User acknowledge theft. Stopping alarm...");
            sendText(User[2], "Acknowledged. Stopping alarm now.");
            lcd.setCursor(0, 2);
            lcd.print(" Resetting Panda...");
            delay(1000);
            clearUser();
            alarmTone(false);
            resetFunc();
            break;
          }
        }

        if (currentTime - prevTime_T3 >= interval_T3) {
          // Also listen for PIN input on the keypad during alarm
          lcd.setCursor(1, 0);
          lcd.print("ENTER PASSWORD");
          unsigned long keypadStartTime = millis();
          if (millis() - keypadStartTime < 60000) {
            char key = keypad.getKey();
            if (key != NO_KEY) {
              delay(60);
              if (key == 'C') {
                resetPassword();
              } else {
                lcd.setCursor(a, 1);
                lcd.print("*");
                a++;
                if (a == 11) {
                  a = 5;
                }
                currentPasswordLength++;
                password.append(key);

                if (currentPasswordLength == maxPasswordLength) {
                  if (password.evaluate()) {
                    Serial.println("PIN entered, stopping alarm...");
                    sendText(User[2], "PIN entered. Stopping alarm now.");
                    lcd.setCursor(1, 2);
                    lcd.print("Resetting Panda...");
                    delay(1000);
                    clearUser();
                    alarmTone(false);
                    alerted = false;
                    resetFunc();
                    break;
                  } else {
                    showError();
                    alarmTone(true);
                  }
                }
              }
            }
          }
          delay(100);
        }
      }
    } else {
      alarmTone(false);
      alerted = false;
    }
  }

  switch (currentState) {
    case SETUP:
      Serial.println("State: SETUP");
      lcd.setCursor(5, 0);
      lcd.print("Welcome to");
      lcd.setCursor(4, 1);
      lcd.print("PARCEL PANDA");
      lcd.setCursor(2, 3);
      lcd.print("Connect to setup;");
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
          notifTone();
          String pinResponse = NewInstance();
          Serial1.println(pinResponse);
          notifTone();
          received = false;
        } else if (receivedData == "Clear") {
          clearUser();
          Serial1.println("Clear Success!");
        }
      }
      break;

    case DELIVERY:
      Serial.println("Waiting for parcel tracking.");
      // currentState = RETRIEVAL;
      // Serial.println("State Switch: RETRIEVAL. (DEBUG PURPOSES)");
      // break;
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
            delay(1000);  // Wait for 1 second before reducing the
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
            delay(1000);  // Wait for 1 second before reducing the
            countdown--;  // Decrease countdown by 1
          }
          monDoor(false);
          received = true;
          isRetrieval = true;
          currentState = RETRIEVAL;
        }
      }

      // currentState = RETRIEVAL;
      // lcd.clear();
      // Serial.println("State Switch: RETRIEVAL");
      // isRetrieval = true;
      // break;

      // case RETRIEVAL:
      // // Step 1: Handle PIN Entry
      // Serial.println("Entered RETRIEVAL case.");
      // // lcd.clear();
      // lcd.setCursor(1, 0);
      // lcd.print("Enter your PIN:");
      // char key = keypad.getKey();
      // if (key != NO_KEY) {
      //   delay(60);
      //   if (key == 'C') {
      //     resetPassword();
      //   } else {
      //     processNumberKey(key);
      //   }
      // }

      // // Step 2: Active Anti-Theft Check
      // if (!isObjectPresent(parCompPins)) {
      //   // If parcel is absent, invoke theft security function
      //   unsigned long startTime = millis();
      //   notifTone();
      //   while (!isObjectPresent(parCompPins)) {
      //     if (millis() - startTime >= 5000) {
      //       lcd.clear();
      //       Serial.println("SECURITY LEVEL 2 ACTIVATED");
      //       SecurityTwo();
      //       while (1) {
      //         if (Serial2.available()) {
      //           String message = Serial2.readString();

      //           if (message.indexOf("STOP") >= 0) {
      //             // User acknowledged the theft and wants to stop the alarma
      //             Serial.println("User acknowledge theft. Stopping
      //             alarm..."); sendText(User[2], "Acknowledged. Stopping alarm
      //             now."); lcd.setCursor(0, 2); lcd.print(" Resetting
      //             Panda..."); delay(1000); clearUser(); alarmTone(false);
      //             resetFunc();
      //             break;
      //           }
      //         }

      //         if (currentTime - prevTime_T3 >= interval_T3) {
      //           // Also listen for PIN input on the keypad during alarm
      //           lcd.setCursor(1, 0);
      //           lcd.print("ENTER PASSWORD");
      //           unsigned long keypadStartTime = millis();
      //           if (millis() - keypadStartTime < 60000) {
      //             char key = keypad.getKey();
      //             if (key != NO_KEY) {
      //               delay(60);
      //               if (key == 'C') {
      //                 resetPassword();
      //               } else {
      //                 lcd.setCursor(a, 1);
      //                 lcd.print("*");
      //                 a++;
      //                 if (a == 11) {
      //                   a = 5;
      //                 }
      //                 currentPasswordLength++;
      //                 password.append(key);

      //                 if (currentPasswordLength == maxPasswordLength) {
      //                   if (password.evaluate()) {
      //                     Serial.println("PIN entered, stopping alarm...");
      //                     sendText(User[2], "PIN entered. Stopping alarm
      //                     now."); lcd.setCursor(1, 2); lcd.print("Resetting
      //                     Panda..."); delay(1000); clearUser();
      //                     alarmTone(false);
      //                     alerted = false;
      //                     resetFunc();
      //                     break;
      //                   } else {
      //                     showError();
      //                     alarmTone(true);
      //                   }
      //                 }
      //               }
      //             }
      //           }
      //           delay(100);
      //         }
      //       }
      //       break;
      //     }

      //     if (isObjectPresent(parCompPins)) {
      //       Serial.println("Condition changed before 5 seconds. Exiting...");
      //       break;
      //     }
      //   }
      // }
      // break;
  }

  if (isRetrieval) {
    // Step 1: Handle PIN Entry
    Serial.println("Entered RETRIEVAL case.");
    // lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Enter your PIN:");
    char key = keypad.getKey();
    if (key != NO_KEY) {
      delay(60);
      if (key == 'C') {
        resetPassword();
      } else {
        processNumberKey(key);
      }
    }

    // Step 2: Active Anti-Theft Check
    if (!isObjectPresent(parCompPins)) {
      // If parcel is absent, invoke theft security function
      unsigned long startTime = millis();
      notifTone();
      while (!isObjectPresent(parCompPins)) {
        if (millis() - startTime >= 5000) {
          lcd.clear();
          Serial.println("SECURITY LEVEL 2 ACTIVATED");
          while (1) {
            SecurityTwo();
            if (Serial2.available()) {
              String message = Serial2.readString();

              if (message.indexOf("STOP") >= 0) {
                // User acknowledged the theft and wants to stop the alarma
                Serial.println("User acknowledge theft. Stopping alarm...");
                sendText(User[2], "Acknowledged. Stopping alarm now.");
                lcd.setCursor(0, 2);
                lcd.print(" Resetting Panda...");
                delay(1000);
                clearUser();
                alarmTone(false);
                resetFunc();
                break;
              }
            }

            if (currentTime - prevTime_T3 >= interval_T3) {
              // Also listen for PIN input on the keypad during alarm
              lcd.setCursor(1, 0);
              lcd.print("ENTER PASSWORD");
              unsigned long keypadStartTime = millis();
              if (millis() - keypadStartTime < 60000) {
                char key = keypad.getKey();
                if (key != NO_KEY) {
                  delay(60);
                  if (key == 'C') {
                    resetPassword();
                  } else {
                    lcd.setCursor(a, 1);
                    lcd.print("*");
                    a++;
                    if (a == 11) {
                      a = 5;
                    }
                    currentPasswordLength++;
                    password.append(key);

                    if (currentPasswordLength == maxPasswordLength) {
                      if (password.evaluate()) {
                        Serial.println("PIN entered, stopping alarm...");
                        sendText(User[2], "PIN entered. Stopping alarm now.");
                        lcd.setCursor(1, 2);
                        lcd.print("Resetting Panda...");
                        delay(1000);
                        clearUser();
                        alarmTone(false);
                        alerted = false;
                        resetFunc();
                        break;
                      } else {
                        showError();
                        alarmTone(true);
                      }
                    }
                  }
                }
              }
              delay(100);
            }
          }
          break;
        }

        if (isObjectPresent(parCompPins)) {
          Serial.println("Condition changed before 5 seconds. Exiting...");
          break;
        }
      }
    }
  }
}