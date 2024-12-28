#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Password.h>
#include <Servo.h>

#define buzzer 13

Servo servo;
LiquidCrystal_I2C lcd(0x27, 20, 4);

byte a = 5;
bool value = true;
bool doorOpen = false;  // Tracks the door state: false = closed, true = open

Password password = Password("123456");  // Default password: 123456

byte maxPasswordLength = 6;
byte currentPasswordLength = 0;
const byte ROWS = 4;  // Four rows
const byte COLS = 4;  // Four columns

char keys[ROWS][COLS] = {{'1', '2', '3', 'A'},
                         {'4', '5', '6', 'B'},
                         {'7', '8', '9', 'C'},
                         {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {30, 31, 32, 33};
byte colPins[COLS] = {34, 35, 36, 37};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
  servo.attach(11);
  servo.write(110);  // Start with the door locked
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
  lcd.print("ENTER PASSWORD");

  char key = keypad.getKey();
  if (key != NO_KEY) {
    delay(60);
    if (key == 'C') {
      resetPassword();
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

  if (currentPasswordLength == maxPasswordLength) {
    evaluatePassword();  // Evaluate and handle the password
  }
}

void evaluatePassword() {
  if (password.evaluate()&&) {
    toggleDoor();  // Toggle door state when password is correct
  } else {
    showError();  // Show an error if the password is incorrect
  }
  resetPassword();  // Reset the password for the next attempt
}

void toggleDoor() {
  if (doorOpen) {
    // Close the door
    digitalWrite(buzzer, HIGH);
    delay(300);
    digitalWrite(buzzer, LOW);
    servo.write(180);  // Move servo to locked position
    lcd.setCursor(0, 0);
    lcd.print("DOOR LOCKED");
    lcd.setCursor(0, 1);
    lcd.print("SECURELY CLOSED");
    doorOpen = false;
  } else {
    // Open the door
    digitalWrite(buzzer, HIGH);
    delay(300);
    digitalWrite(buzzer, LOW);
    servo.write(0);  // Move servo to open position
    lcd.setCursor(0, 0);
    lcd.print("DOOR OPENED");
    lcd.setCursor(0, 1);
    lcd.print("WELCOME IN!");
    doorOpen = true;
  }
  delay(2000);
  lcd.clear();
  a = 5;
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
  lcd.clear();
  a = 5;
}
