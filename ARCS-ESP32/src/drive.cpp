
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <PID.h>

BluetoothSerial BS;
//#include <HardwareSerial.h>
PID pidController = PID();
PID leftDrivePID = PID();
PID rightDrivePID = PID();

double kP = 0.45;
double kI = 0.0035;
double kD = 0.0;
// Function prototypes because c++ is a liar
void stopAllMotors();
void directionForward();
void directionBackward();
void setSpeed(int speedA, int speedB);
void updateEncoderLeft();
void updateEncoderRight();
void driveDistance(float distance, int speed);
void turnDegrees(float degrees, int speed);
void forwards();
void backwards();
void left();
void right();
long readEncoderLeft();
long readEncoderRight();
long distanceToEncoderTicks(float distanceMm);
int pidOutputToSpeed(double output, long error, int maxSpeed);
bool runToEncoderTargets(long leftTargetTicks, long rightTargetTicks, int speed);

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

long targetPositionRight;
long targetPositionLeft;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

// Encoder Connections
const int ENCAFL = 35; // Encoder A pin for Front Left Motor
const int ENCBFL = 26; // Encoder B pin for Front Left Motor

const int ENCAFR = 33; // Encoder A pin for Front Right Motor
const int ENCBFR = 25; // Encoder B pin for Front Right Motor

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

int powerValue = 1100;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float trackWidth = 330.0; // mm - TODO
const float turnSlipCompensation = 1.64; // This is a fudge factor to account for the fact that the robot doesn't turn perfectly in place
const float movementSpeed = 128.0; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 178.0; // Speed for turning left/right (0-255) - TODO
const float DISTANCE_PER_TICK = (PI * wheelDiameter) / ticksPerRotation; // mm per encoder tick
const int encoderToleranceTicks = 15;
const int minimumPIDSpeed = 55;
const unsigned long movementLoopDelayMs = 10;

const float cameraFOVWidthMM = 100.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 75.0; // Height of the camera's field of view in millimeters - TODO

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

  pinMode(ENCAFL, INPUT);
  pinMode(ENCBFL, INPUT);
  pinMode(ENCAFR, INPUT);
  pinMode(ENCBFR, INPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);
  
  pidController.Init(kP, kI, kD);
  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);
  // Turn off motors initially
  stopAllMotors();
}

// MARK: Loop
long encoderStartLeft = 0;
long encoderStartRight = 0;
long encoderEndLeft = 0;
long encoderEndRight = 0;
float rpmLeft = 0.0;
float rpmRight = 0.0;
int pwmValue = 255;

void loop() {
  // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

  // Serial.print(String(encoderValueLeft));
  // Serial.print(", ");
  // Serial.println(String(encoderValueRight));

  // driveDistance(44 * 2 * PI, 255);

  turnDegrees(180 , 200);
  // turnDegrees2(90);

  // forwards();
  // for (pwmValue = 255; pwmValue >= 0; pwmValue -= 17) {
  //   encoderStartLeft = encoderValueLeft;
  //   encoderStartRight = encoderValueRight;
  //   setSpeed(pwmValue, pwmValue);
  //   delay(5000);
  //   stopAllMotors();
  //   encoderEndLeft = encoderValueLeft;
  //   encoderEndRight = encoderValueRight;
  //   rpmLeft = (encoderEndLeft - encoderStartLeft) / 5.0 * 60.0 / ticksPerRotation;
  //   rpmRight = (encoderEndRight - encoderStartRight) / 5.0 * 60.0 / ticksPerRotation;
  //   Serial.print("PWM Value: ");
  //   Serial.print(pwmValue);
  //   Serial.print(", RPM Left: ");
  //   Serial.print(rpmLeft);
  //   Serial.print(", RPM Right: ");
  //   Serial.println(rpmRight);
  //   delay(5000);
  // }

  // setSpeed(255, 255);


  // left();
  while (true) {}

  // Serial.print(String(digitalRead(ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));
}

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

// MARK: Encoder Functions

long readEncoderLeft() {
  noInterrupts();
  long value = encoderValueLeft;
  interrupts();
  return value;
}

long readEncoderRight() {
  noInterrupts();
  long value = encoderValueRight;
  interrupts();
  return value;
}

long distanceToEncoderTicks(float distanceMm) {
  return lround((distanceMm / wheelCircumference) * ticksPerRotation);
}

int pidOutputToSpeed(double output, long error, int maxSpeed) {
  if (abs(error) <= encoderToleranceTicks) {
    return 0;
  }

  int pwm = abs((int)round(output));
  if (pwm > maxSpeed) {
    pwm = maxSpeed;
  }

  int speedFloor = min(minimumPIDSpeed, maxSpeed);
  if (pwm < speedFloor) {
    pwm = speedFloor;
  }

  return error >= 0 ? pwm : -pwm;
}

// MARK: PID Logic ish
bool runToEncoderTargets(long leftTargetTicks, long rightTargetTicks, int speed) {
  int maxSpeed = constrain(abs(speed), 0, 255);
  if (maxSpeed == 0 || (leftTargetTicks == 0 && rightTargetTicks == 0)) {
    stopAllMotors();
    return true;
  }

  long startLeft = readEncoderLeft();
  long startRight = readEncoderRight();
  targetPositionLeft = startLeft + leftTargetTicks;
  targetPositionRight = startRight + rightTargetTicks;

  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);

  float longestDistanceMm = (max(abs(leftTargetTicks), abs(rightTargetTicks)) / (float)ticksPerRotation) * wheelCircumference;
  unsigned long timeoutMs = (unsigned long)((longestDistanceMm / mmPerSecond(maxSpeed)) * 3000.0) + 1000;
  unsigned long startTime = millis();

  while (true) {
    long currentLeft = readEncoderLeft();
    long currentRight = readEncoderRight();
    long leftError = targetPositionLeft - currentLeft;
    long rightError = targetPositionRight - currentRight;

    bool leftAtTarget = abs(leftError) <= encoderToleranceTicks;
    bool rightAtTarget = abs(rightError) <= encoderToleranceTicks;

    // Serial.print("Left Error: ");
    // Serial.print(leftError);
    // Serial.print(", Right Error: ");
    // Serial.print(rightError);
    // Serial.print(", Left Target: ");
    // Serial.print(targetPositionLeft);   
    // Serial.print(", Right Target: ");
    // Serial.print(targetPositionRight);
    // Serial.print(", Left Current: ");
    // Serial.print(currentLeft);
    // Serial.print(", Right Current: ");
    // Serial.print(currentRight);

    if (leftAtTarget && rightAtTarget) {
      stopAllMotors();
      return true;
    }

    if (millis() - startTime > timeoutMs) {
      stopAllMotors();
      Serial.println("Encoder movement timed out");
      return false;
    }
    
    leftDrivePID.UpdateError(leftError);
    rightDrivePID.UpdateError(rightError);
    // Serial.print(", Left P: ");
    // Serial.print(leftDrivePID.p_error);
    // Serial.print(", Left I: ");
    // Serial.print(leftDrivePID.i_error);
    // Serial.print(", Left D: ");
    // Serial.print(leftDrivePID.d_error);
    // Serial.print(", Right P: ");
    // Serial.print(rightDrivePID.p_error);
    // Serial.print(", Right I: ");
    // Serial.print(rightDrivePID.i_error);
    // Serial.print(", Right D: ");
    // Serial.println(rightDrivePID.d_error);
    double leftOutput = leftDrivePID.TotalError();
    double rightOutput = rightDrivePID.TotalError();

    // Serial.print(", Left Output: ");
    // Serial.print(leftOutput);
    // Serial.print(", Right Output: ");
    // Serial.println(rightOutput);

    int leftSpeed = leftAtTarget ? 0 : pidOutputToSpeed(leftOutput, leftError, maxSpeed);
    int rightSpeed = rightAtTarget ? 0 : pidOutputToSpeed(rightOutput, rightError, maxSpeed);
    if (leftTargetTicks == -rightTargetTicks) {
      // If turning, ensure both motors are moving at the same speed
      setSpeed(leftSpeed, -leftSpeed);
      Serial.print("Output Speed:");
      Serial.println(leftSpeed);
    } else {
      setSpeed(leftSpeed, rightSpeed);
    }
    delay(movementLoopDelayMs);
  }
}

void driveDistance(float distance, int speed) {
  long targetTicks = distanceToEncoderTicks(distance);
  runToEncoderTargets(targetTicks, targetTicks, speed);
}

void turnDegrees2(float theta) {
  int startLeft = encoderValueLeft; // starting point for encoders
  int startRight = encoderValueRight;

  Serial.print("Starting Encoders: Left = ");
  Serial.println(startLeft);
  Serial.print(", Right = ");
  Serial.println(startRight);

  float currentAngle = 0; // Reset current angle
  float angleTolerance = 1.0; // Tolerance in degrees for stopping the turn

  // 3. Keep moving until the target is reached
  while (abs(currentAngle-theta) > abs(angleTolerance)) {
    // Direct motors based on target direction
    Serial.print(" Current Angle: ");
    Serial.println(currentAngle);
    if (theta > 0) {
        left();  // Turn left
    } else {
        right(); // Turn right
    }
    
    // 4. Calculate real-time distance traveled by each wheel
    float leftDistance = (encoderValueLeft - startLeft) * DISTANCE_PER_TICK;
    float rightDistance = (encoderValueRight - startRight) * DISTANCE_PER_TICK;
    
    // 5. Update the current angle dynamically
    currentAngle = ((leftDistance - rightDistance) / trackWidth) * (180.0 / PI);

  } 
}

void turnDegrees(float degrees, int speed) {
  float wheelTravelMm = trackWidth * turnSlipCompensation * PI * (degrees / 360.0);
  long targetTicks = distanceToEncoderTicks(wheelTravelMm);
  runToEncoderTargets(targetTicks, -targetTicks, speed);
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

  Serial.print("Driving to crack center: distance = ");
  Serial.print(distance);
  Serial.print(" mm, angle = ");
  Serial.print(angle);
  Serial.println(" degrees");

  // Rotate to face the crack center
  turnDegrees(angle, turnSpeed);
  
  // Drive forward to the crack center
  driveDistance(distance, movementSpeed);
}



// MARK: Movement Functions

void forwards() {
  setSpeed(movementSpeed, movementSpeed);
}

void backwards() {
  setSpeed(-movementSpeed, -movementSpeed);
}

void left() {
  setSpeed(-turnSpeed, turnSpeed);
}

void right() {
  setSpeed(turnSpeed, -turnSpeed);
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
    encoderValueLeft--;
  else
    encoderValueLeft++;
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


 
