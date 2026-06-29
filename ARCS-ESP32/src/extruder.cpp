#include <Arduino.h>
#include <PID.h>
#include <Adafruit_VL6180X.h>
#include "extruder.h"

//Gantry pins
const int ENCODER_PIN_A1 = 0;
const int ENCODER_PIN_B1 = 0;
const int PWM_PIN1 = 27;
const int ENI = 14;
const int PWM_PIN2 = 26; 

const int STANDARD_DISTANCE = 0;

int depth;

//Spray pins
// const int PWM_PIN2 = 0;
PID pidController = PID();

const int kP = 0;
const int kI = 0;
const int kD = 0;

int motorPosition = 0; 

int targetPosition = 0;

Adafruit_VL6180X ir = Adafruit_VL6180X();

int encoderAValue;
int encoderBValue;

void setMotorPosition(int targetPos);
bool isAtTargetPosition();
void spray();

bool isFilledFully();

void setup() {
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(ENI, OUTPUT);
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP);

  Serial.begin(115200);

//   ir.begin();
//   depth = ir.readRange();

//   pidController.Init(kP, kI, kD); 
}

void loop() {
    // encoderAValue = digitalRead(ENCODER_PIN_A1);
    // encoderBValue = digitalRead(ENCODER_PIN_B1);

    // if (encoderAValue == HIGH && encoderBValue == LOW) {
    //     motorPosition++;
    // } else if (encoderAValue == LOW && encoderBValue == HIGH) {
    //     motorPosition--;
    // }

    analogWrite(ENI, 100);
    digitalWrite(PWM_PIN1, LOW);
    digitalWrite(PWM_PIN2, HIGH);
    // depth = ir.readRange();
}

void setMotorPosition(int targetPos) {
    int error = targetPos - motorPosition;
    targetPosition = targetPos;

    pidController.UpdateError(error);

    analogWrite(PWM_PIN1, pidController.p_error*kP + pidController.i_error*kI + pidController.d_error*kD);    
}

bool isAtTargetPosition() {
    return abs(targetPosition - motorPosition) < 2;
}

void spray(){
    analogWrite(ENI, 100);
    digitalWrite(PWM_PIN1, LOW);
    digitalWrite(PWM_PIN2, HIGH);

}

bool isFilledFully(){
    return abs(STANDARD_DISTANCE - depth) < 4;
}
