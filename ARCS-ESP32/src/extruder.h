#include "PID.h"
#include <Adafruit_VL6180X.h>
#ifndef extruder_h
#define extruder_h

class extruder{
public:
    int ENCODER_PIN_A1;
    int ENCODER_PIN_B1 = 0;
    int PWM_PIN1 = 0; 
    int STANDARD_DISTANCE = 0;
    int depth;

    int PWM_PIN2 = 0;
    PID pidController;

    int kP;
    int kI;
    int kD;

    int motorPosition; 

    int targetPosition;

    Adafruit_VL6180X ir;

    int encoderAValue;
    int encoderBValue;

    void setup();

    void loop();

    void setMotorPosition(int targetPosition);

    bool isAtTargetPosition();

    void spray();

    bool isFilledFully();
};


#endif 