#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// ! VARIABLE & PIN DEFINITIONS
#define BUZZER_PIN 13
#define CSpin 53
bool availability = true;
String User[4] = {"TRACKING", "+639915176440", "false", "123456"};

// ! CLASSES DEFINITIONS
File myFile;

// buzzer tones
const uint16_t success[] PROGMEM = {100, 100, 100, 100};
const uint16_t error[] PROGMEM = {300, 100, 100, 100};
const uint16_t alarm[] PROGMEM = {100, 100, 100, 100, 500};
const uint16_t notif[] PROGMEM = {100};

// ! FUNCTION DEFINITIONS
bool isValidPin(String data);

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
void alarmTone() { playPattern(alarm, sizeof(alarm) / sizeof(alarm[0])); }
void notifTone() { playPattern(notif, sizeof(notif) / sizeof(notif[0])); }

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
    Serial.println("Data read successfully.");
  } else {
    Serial.println("Error opening USERINFO.txt for reading.");
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
    } else {
      Serial.println("Error opening PREVUSERS.txt");
      readFile.close();
    }
  } else {
    Serial.println("Error opening USERINFO.txt");
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CSpin, OUTPUT);
  Serial.begin(9600);
  Serial1.begin(9600);
  randomSeed(analogRead(10));

  if (!SD.begin(CSpin)) {
    Serial.println("SD card initialization failed.");
    while (true);
  }
  Serial.println("SD card is ready to use.");
}

void loop() {
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
      String readData[4];
      readUser(readData);
      return readData[3];
    } else {
      return "000000";
    }
  } else {
    Serial.println("Timeout reached: PIN not received in time or invalid PIN.");
    return "000000";
  }
}
