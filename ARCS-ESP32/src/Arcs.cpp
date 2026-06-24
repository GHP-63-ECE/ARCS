#include <Arduino.h>
// Motor A Connections
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Motor B Connections
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// Motor C Connections
const int ENC = 9;
const int IN5 = 8;
const int IN6 = 7;

// Motor D Connections
const int END = 3;
const int IN7 = 5;
const int IN8 = 4;


void setup() {
  // Set all control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Turn off motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
}

void loop() {
  // Move both motors forward at maximum speed (255)
  directionForward();
  setSpeed(255, 255);
  delay(2000);
  
  // Stop motors for 1 second
  stopMotors();
  delay(1000);
  
  // Move both motors backward at half speed (127)
  directionBackward();
  setSpeed(127, 127);
  delay(2000);
  
  // Stop motors
  stopMotors();
  delay(1000);
}

// Function to run motors forward
void directionForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to run motors backward
void directionBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Function to set individual motor speeds (PWM values: 0 to 255)
void setSpeed(int speedA, int speedB) {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}
