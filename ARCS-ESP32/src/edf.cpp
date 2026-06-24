#include <Arduino.h>

const int EDF_PIN = 4; // Pin connected to the EDF motor controller

void setup() {
  pinMode(EDF_PIN, OUTPUT); // Set the EDF pin as an output
}

void loop() {
  analogWrite(EDF_PIN, 128); // Set the EDF motor speed (0-255)
}