#ifndef drive_h
#include <Arduino.h>
#define drive_h
class drive{
public:
  int PWMFL;
  int FL1;
  int FL2;

  // Front Right
  int PWMFR;
  int FR1;
  int FR2;

  // Back Left
  int PWMBL;
  int BL1;
  int BL2;

  // Back Right
  int PWMBR;
  int BR1;
  int BR2;

  // Encoder Connections
  int ENCAFL; // Encoder A pin for Front Left Motor
  int ENCBFL; // Encoder B pin for Front Left Motor

  int ENCAFR; // Encoder A pin for Front Right Motor
  int ENCBFR; // Encoder B pin for Front Right Motor

  long encoderValueLeft;
  long encoderValueRight;

  float wheelDiameter; // mm
  long ticksPerRotation; 
  float circumference;


  // Function to run motors forward
  void directionForward();

  // Function to run motors backward
  void directionBackward();

  // Function to stop both motors
  void stopAllMotors();

  // Function to set individual motor speeds (PWM values: 0 to 255)
  void setSpeed(int speedA, int speedB);

  void updateEncoderLeft();

  void updateEncoderRight();
};
#endif