#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <PID.h>

BluetoothSerial BS;
//#include <HardwareSerial.h>
PID pidController = PID();

double kP = 0;
double kI = 0;
double kD = 0;
// Function prototypes because c++ is a liar
void stopAllMotors();
void directionForward();
void directionBackward();
void setSpeed(int speedA, int speedB);
void updateEncoderLeft();
void updateEncoderRight();
void driveDistance(float distance, int speed);
void forwards();
void backwards();
void left();
void right();

//Raspberry Pi communication Pin Definitions
//#define RXD2 16  // GPIO16 as RX
//#define TXD2 17  // GPIO17 as TX

//HardwareSerial mySerial(2); // Use Serial2
 
// Motor Pins
// Left
const int PWML = 23;
const int L1 = 22;
const int L2 = 21;

// Right
const int PWMR = 19;
const int R1 = 18;
const int R2 = 5;

int targetPositionRight;
int targetPositionLeft;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

// Encoder Connections
const int ENCAFL = 35; // Encoder A pin for Front Left Motor
const int ENCBFL = 34; // Encoder B pin for Front Left Motor

const int ENCAFR = 33; // Encoder A pin for Front Right Motor
const int ENCBFR = 32; // Encoder B pin for Front Right Motor

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

int powerValue = 1100;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float trackWidth = 150.0; // mm - TODO
const float movementSpeed = 128.0; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 128.0; // Speed for turning left/right (0-255) - TODO

const float cameraFOVWidthMM = 100.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 75.0; // Height of the camera's field of view in millimeters - TODO

// MARK: Movement Calculations

float RPM(float speed) {
  return rpmAtMaxSpeed * (speed / 255.0);
}

float mmPerSecond(float speed) {
  return (RPM(speed) * wheelCircumference) / 60;
}

float degreesPerSecond(float speed) {
  return (2 * mmPerSecond(speed) / trackWidth) * 180 / PI;
}

void driveDistanceWithoutEncoders(float distance, int speed) {
  float timeToDrive = distance / mmPerSecond(speed);
  timeToDrive = abs(timeToDrive); // Ensure time is positive
  setSpeed(speed, speed);
  delay(timeToDrive * 1000);
  stopAllMotors();
}

void rotateDegreesWithoutEncoders(float degrees, int speed) {
  float timeToRotate = degrees / degreesPerSecond(speed);
  timeToRotate = abs(timeToRotate); // Ensure time is positive
  if (degrees > 0) {
    setSpeed(speed, -speed); // Turn right
  } else {
    setSpeed(-speed, speed); // Turn left
  }
  delay(timeToRotate * 1000);
  stopAllMotors();
}

// Calculating distance to the center of a crack based on normalized coordinates (cx, cy) of the crack in the camera's field of view
float distanceToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - 0.5) * cameraFOVWidthMM;
  float y_mm = (cy - 0.5) * cameraFOVHeightMM;

  // Calculate distance to the center of the crack using Pythagorean theorem
  return sqrt(x_mm * x_mm + y_mm * y_mm);
}

float angleToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - 0.5) * cameraFOVWidthMM;
  float y_mm = (cy - 0.5) * cameraFOVHeightMM;

  // Calculate angle to the center of the crack using arctangent
  return atan2(x_mm, -y_mm) * (180 / PI); // atan2 is flipped so that 0 degrees is forward and positive angles are to the right
}

// Drives to the crack's center such that the crack is centered in the camera's field of view at (0.5, 0.5) 
void driveToCrackCenter(float cx, float cy) {
  float distance = distanceToCrackCenter(cx, cy);
  float angle = angleToCrackCenter(cx, cy);

  // Rotate to face the crack center
  rotateDegreesWithoutEncoders(angle, turnSpeed);

  // Drive forward to the crack center
  driveDistanceWithoutEncoders(distance, movementSpeed);
}

// MARK: Setup

void setup() {

  Serial.begin(115200);

  // Set all control pins to outputs

  //  Left
  pinMode(PWML, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  //  Right
  pinMode(PWMR, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);
  
  pidController.Init(kP, kI, kD);
  // Turn off motors initially
  stopAllMotors();
}

// MARK: Loop

void loop() {
  // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

  Serial.print(String(encoderValueLeft));
  Serial.print(", ");
  Serial.println(String(encoderValueRight));
  

  // Serial.print(String(digitalRead(ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));
}



// MARK: Movement Functions

void forwards() {
  setSpeed(movementSpeed, movementSpeed);
}

void backwards() {
  setSpeed(-movementSpeed, -movementSpeed);
}

void left() {
  setSpeed(-movementSpeed, movementSpeed);
}

void right() {
  setSpeed(movementSpeed, -movementSpeed);
}

void stopLeftMotors() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(PWML, 0);
}

void stopRightMotors() {
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
  digitalWrite(PWMR, 0);
}

void stopAllMotors() {
  stopLeftMotors();
  stopRightMotors();
}

void LeftMotorsForwards() {
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
}

void RightMotorsForwards() {
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
}

void LeftMotorsBackwards() {
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
}

void RightMotorsBackwards() {
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void setSpeed(int left, int right){
  if (left == 0) {
    stopLeftMotors();
  } else if (left < 0) {
    LeftMotorsBackwards();
    left = -left;
  } else {
    LeftMotorsForwards();
  }

  if (right == 0) {
    stopRightMotors();
  } else if (right < 0) {
    RightMotorsBackwards();
    right = -right;
  } else {
    RightMotorsForwards();
  }
  analogWrite(PWML, left);
  analogWrite(PWMR, right);
}

// MARK: Encoder Functions

void updateEncoderLeft(){
  if (digitalRead(ENCAFL) > digitalRead(ENCBFL))
    encoderValueLeft++;
  else
    encoderValueLeft--;
}

void updateEncoderRight(){
  if (digitalRead(ENCAFR) > digitalRead(ENCBFR))
    encoderValueRight++;
  else
    encoderValueRight--;
}

// MARK: PID Functions

void setMotorPositionLeft(int targetPos) {
    int error = targetPos - encoderValueLeft;
    targetPositionLeft = targetPos;

    pidController.UpdateError(error);

    analogWrite(PWML, pidController.p_error*kP + pidController.i_error*kI + pidController.d_error*kD);    
}

bool isAtTargetPositionLeft() {
    return abs(targetPositionLeft - encoderValueLeft) < 2;
}

void setMotorPositionRight(int targetPos) {
    int error = targetPos - encoderValueRight;
    targetPositionRight = targetPos;

    pidController.UpdateError(error);

    analogWrite(PWMR, pidController.p_error*kP + pidController.i_error*kI + pidController.d_error*kD);    
}

bool isAtTargetPositionRight() {
    return abs(targetPositionRight - encoderValueRight) < 2;
}