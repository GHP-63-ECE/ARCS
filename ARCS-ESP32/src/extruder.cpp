#include <Arduino.h>
#include <PID.h>

//Gantry pins
const int ENCODER_PIN_A1 = 0;
const int ENCODER_PIN_B1 = 0;
const int PWM_PIN1 = 0; 

//Spray pins
const int ENCODER_PIN_A2 = 0;
const int ENCODER_PIN_B2 = 0;
const int PWM_PIN2 = 0;

PID pidController = PID();

const int kP = 0;
const int kI = 0;
const int kD = 0;

int motorPosition = 0; 

int targetPosition = 0;

int encoderAValue;
int encoderBValue;

void setup() {
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);

  Serial.begin(115200);

  pidController.Init(kP, kI, kD); 

}

void loop() {
    encoderAValue = digitalRead(ENCODER_PIN_A1);
    encoderBValue = digitalRead(ENCODER_PIN_B1);

    if (encoderAValue == HIGH && encoderBValue == LOW) {
        motorPosition++;
    } else if (encoderAValue == LOW && encoderBValue == HIGH) {
        motorPosition--;
    }
}

void setMotorPosition(int targetPostion) {
    int error = targetPostion - motorPosition;

    pidController.UpdateError(error);

    analogWrite(PWM_PIN1, pidController.p_error*kP + pidController.i_error*kI + pidController.d_error*kD);    
}

bool isAtTargetPosition() {
    return abs(targetPosition - motorPosition) < 2;
}

void spray(){
    analogWrite(PWM_PIN2, 255);
}
