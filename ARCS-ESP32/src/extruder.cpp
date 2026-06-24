#include <Arduino.h>
#include <PID.h>

const int ENCODER_PIN_A = 2; // Pin connected to encoder channel A
const int ENCODER_PIN_B = 3; // Pin connected to encoder channel B

PID pidController = PID();

const int kP = 0;
const int kI = 0;
const int kD = 0;

int motorPosition = 0; 

int encoderAValue;
int encoderBValue;

void setup() {
  pinMode(4, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  

  Serial.begin(115200);

  pidController.Init(kP, kI, kD); 

}

void loop() {
    encoderAValue = digitalRead(ENCODER_PIN_A);
    encoderBValue = digitalRead(ENCODER_PIN_B);

    if (encoderAValue == HIGH && encoderBValue == LOW) {
        motorPosition++;
    } else if (encoderAValue == LOW && encoderBValue == HIGH) {
        motorPosition--;
    }
}

void setMotorPosition(int targetPostion) {
    int error = targetPostion - motorPosition;

    pidController.UpdateError(error);

    
}