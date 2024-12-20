#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Password.h>
#include <Servo.h>

#define buzzer 13

Servo servo;
LiquidCrystal_I2C lcd(0x27, 20, 4);

String newPasswordString;  // Hold the new password
char newPassword[7];  // Character string of newPasswordString (6 digits + null
                      // terminator)
byte a = 5;
bool value = true;
bool isChangingPassword = false;      // Track password change mode
bool isVerifyingOldPassword = false;  // Verify old password

Password password = Password("123456");  // Default password: 123456

byte maxPasswordLength = 6;
byte currentPasswordLength = 0;
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns

char keys[ROWS][COLS] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
  servo.attach(11);
  servo.write(50);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("WELCOME TO");
  lcd.setCursor(0, 1);
  lcd.print("DOOR LOCK SYSTEM");
  delay(3000);
  lcd.clear();
}

void loop() {
  lcd.setCursor(1, 0);
  if (isChangingPassword) {
    if (isVerifyingOldPassword) {
      lcd.print("ENTER OLD PASSWORD");
    } else {
      lcd.print("SET NEW PASSWORD");
    }
  } else {
    lcd.print("ENTER PASSWORD");
  }

  char key = keypad.getKey();
  if (key != NO_KEY) {
    delay(60);
    if (key == 'C') {
      resetPassword();
    } else if (key == 'D') {
      if (isChangingPassword) {
        cancelPasswordChange();
      } else {
        startPasswordChange();
      }
    } else {
      processNumberKey(key);
    }
  }
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

  if (isChangingPassword && isVerifyingOldPassword) {
    // Collect old password during verification
    if (currentPasswordLength == maxPasswordLength) {
      verifyOldPassword();
    }
  } else if (isChangingPassword && !isVerifyingOldPassword) {
    // Collect new password
    newPasswordString += key;
    if (currentPasswordLength == maxPasswordLength) {
      finalizePasswordChange();
    }
  } else {
    // Normal password evaluation
    if (currentPasswordLength == maxPasswordLength) {
      doorlocked();
      dooropen();
    }
  }
}

void dooropen() {
  if (password.evaluate()) {
    digitalWrite(buzzer, HIGH);
    delay(300);
    digitalWrite(buzzer, LOW);
    servo.write(50);
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("CORRECT PASSWORD");
    lcd.setCursor(0, 1);
    lcd.print("OPEN THE DOOR...");
    delay(2000);
    lcd.clear();
    a = 5;
  } else {
    showError();
  }
  resetPassword();
}

void doorlocked() {
  if (password.evaluate()) {
    digitalWrite(buzzer, HIGH);
    delay(300);
    digitalWrite(buzzer, LOW);
    servo.write(110);
    delay(100);
    lcd.setCursor(0, 0);
    lcd.print("CORRECT PASSWORD");
    lcd.setCursor(2, 1);
    lcd.print("DOOR LOCKED");
    delay(2000);
    lcd.clear();
    a = 5;
  } else {
    showError();
  }
  resetPassword();
}

void showError() {
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
  delay(200);
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
  delay(200);
  digitalWrite(buzzer, HIGH);
  delay(200);
  digitalWrite(buzzer, LOW);
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
  newPasswordString = "";  // Reset new password string
  lcd.clear();
  a = 5;
}

void startPasswordChange() {
  resetPassword();
  isChangingPassword = true;
  isVerifyingOldPassword = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ENTER OLD PASS:");
}

void verifyOldPassword() {
  if (password.evaluate()) {
    isVerifyingOldPassword = false;
    resetPassword();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SET NEW PASS:");
  } else {
    showError();
    cancelPasswordChange();
  }
}

void finalizePasswordChange() {
  if (newPasswordString.length() == maxPasswordLength) {
    newPasswordString.toCharArray(
        newPassword, maxPasswordLength + 1);  // Convert string to char array
    password.set(newPassword);                // Set the new password
    isChangingPassword = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Password changed");
    delay(2000);
    resetPassword();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("INVALID PASSWORD");
    lcd.setCursor(0, 1);
    lcd.print("MUST BE 6 DIGITS");
    delay(2000);
    startPasswordChange();
  }
}

void cancelPasswordChange() {
  isChangingPassword = false;
  isVerifyingOldPassword = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CHANGE CANCELED");
  delay(2000);
  resetPassword();
}
