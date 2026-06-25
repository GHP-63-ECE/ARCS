#include <Arduino.h>

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
  analogWrite(PWMBL, speedA);
  analogWrite(PWMBR, speedB);
}

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

