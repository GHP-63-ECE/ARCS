#include <Arduino.h>
// Motor Pins

// Front Left
const int PWMFL = 9;
const int FL1 = 8;
const int FL2 = 7;

// Front Right
const int PWMFR = 3;
const int FR1 = 5;
const int FR2 = 4;

// Back Left
const int PWMBL = 9;
const int BL1 = 8;
const int BL2 = 7;

// Back Right
const int PWMBR = 3;
const int BR1 = 5;
const int BR2 = 4;

// Encoder Connections
const int ENCAFL = 2; // Encoder A pin for Motor A
const int ENCBFL = 3; // Encoder B pin for Motor A

const int ENCAFR = 3; // Encoder A pin for Motor B
const int ENCBFR = 4; // Encoder B pin for Motor B

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float circumference = wheelDiameter * PI;

void updateEncoderLeft() {
  if (digitalRead(ENCAFL)> digitalRead(ENCBFL))
    encoderValueLeft++;
  else
    encoderValueLeft--;
}

void updateEncoderRight() {
  if (digitalRead(ENCAFR)> digitalRead(ENCBFR))
    encoderValueRight++;
  else
    encoderValueRight--;
}

// Function to run motors forward
void directionForward() {
  digitalWrite(FL1, HIGH);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
}

// Function to run motors backward
void directionBackward() {
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, HIGH);
}

// Function to stop both motors
void stopAllMotors() {
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR1, LOW);
  digitalWrite(BR2, LOW);
}

// Function to set individual motor speeds (PWM values: 0 to 255)
void setSpeed(int speedA, int speedB) {
  analogWrite(PWMFL, speedA);
  analogWrite(PWMFR, speedB);
  analogWrite(PWMBL, speedB);
  analogWrite(PWMBR, speedB);
}


/// MARK: Setup
void setup() {
  // Set all control pins to outputs

  // Front Left
  pinMode(PWMFL, OUTPUT);
  pinMode(FL1, OUTPUT);
  pinMode(FL2, OUTPUT);
  
  // Front Right
  pinMode(PWMFR, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  
  // Back Left
  pinMode(PWMBL, OUTPUT);
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  
  // Back Right
  pinMode(PWMBR, OUTPUT);
  pinMode(BR1, OUTPUT);
  pinMode(BR2, OUTPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFR), updateEncoderRight, RISING);
  
  // Turn off motors initially
  stopAllMotors();
}


/// MARK: Main Loop
void loop() {
  // Move both motors forward at maximum speed (255)
  directionForward();
  setSpeed(255, 255);
  delay(2000);
  
  // Stop motors for 1 second
  stopAllMotors();
  delay(1000);
  
  // Move both motors backward at half speed (127)
  directionBackward();
  setSpeed(127, 127);
  delay(2000);
  
  // Stop motors
  stopAllMotors();
  delay(1000);
}
