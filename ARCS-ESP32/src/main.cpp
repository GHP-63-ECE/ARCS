#include <Arduino.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <BluetoothSerial.h>
#include <PID.h>
#include <esp_now.h>
#include <WiFi.h>
#include <array>
#include <vector>

// MARK: Function prototypes 
// because c++ is a liar

void stopAllDriveMotors();
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
long readEncoderGantry();
long distanceToEncoderTicks(float distanceMm);
int pidOutputToSpeed(double output, long error, int maxSpeed);
bool runToEncoderTargets(long leftTargetTicks, long rightTargetTicks, int speed);
void setGantryPower(int power);
void updateEncoderGantry();
bool setGantryPosition(int targetPos, int maxSpeed);
int readEncoderExtruder();

// MARK: Pinout

// Left Motor Pins
const int PWMR = 23; 
const int R1 = 21; // const int L1 = 22;
const int R2 = 22; // const int L2 = 21;

// Right Motor Pins
const int PWML = 19; 
const int L1 = 18;
const int L2 = 5;

// Gantry Motor Pins
const int ENI = 25;
const int Gantry1 = 27;
const int Gantry2 = 26; 

// Extruder Motor Pins
const int PWM_EXT = 14;
const int EXT1 = 4;
const int EXT2 = 12;

// Encoder Connections
const int ENCAFL = 34; // Encoder A pin for Front Left Motor
const int ENCBFL = 35; // Encoder B pin for Front Left Motor

const int ENCAFR = 32; // Encoder A pin for Front Right Motor
const int ENCBFR = 33; // Encoder B pin for Front Right Motor

const int ENCGANA = 36; // 1st Encoder Pin for Gantry
const int ENCGANB = 39; // 2nd Encoder Pin for Gantry 

const int ENCEXTA = 2;
const int ENCEXTB = 15;


byte servoPin = 13; // signal pin for the ESC.

Servo servo;
PID pidController = PID();
PID leftDrivePID = PID();
PID rightDrivePID = PID();
PID pidControllerGantry = PID();
PID pidControllerExt = PID();
BluetoothSerial BS;

// MARK: Variables

float angle = 0;

// Drivetrain PID values
double kP = 0.45;
double kI = 0.0035; 
double kD = 0.0; // we dont need a big d

double kP_gantry = 0.1;
double kI_gantry = 0;
double kD_gantry = 0;

double kP_ext = 0.1;
double kI_ext = 0;
double kD_ext = 0;

float cx = 0.85;
float cy = 0.2;
float ax[5] = {0.1,0.3,0.5,0.7,0.9};
float ay[5] = {0.2, 0.25, 0.45, 0.7, 0.8};

bool autonomous = false;
bool prevButton4State = false;
bool prevButton5State = false;

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;
volatile long encoderValueGantry = 0;
volatile long encoderValueExtruder = 0;

int targetPositionRight;
int targetPositionLeft;

int targetPositionGantry;
int minGantryPosition = 0;
int maxGantryPosition = 9996;
float gantryTicksPerMM = 9996 / 185.7;
bool hasZeroedGantry = false;

int targetPositionExtruder;
int minExtruderPosition = 0;
int maxExtruderPosition = 0;
bool hasZeroedExtruder;

const float wheelDiameter = 45.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float turnSlipCompensation = 1.15; // This is a fudge factor to account for the fact that the robot doesn't turn perfectly in place
const float trackWidth = 350.0; // mm
int movementSpeed = 255; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 200; // Speed for turning left/right (0-255) - TODO
const int encoderToleranceTicks = 15;
const int minimumPIDSpeed = 55;
const unsigned long movementLoopDelayMs = 10;

const float cameraFOVWidthMM = 95.0; // Width of the camera's field of view in millimeters 
const float cameraFOVHeightMM = 70.0; // Height of the camera's field of view in millimeters 

int pwrGantry = 100;
int pwrExt = 100;

// MARK: Structs

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
int vy1;
int vy2;
int vy3;
int vx2;
bool button1;
bool button2;
bool button3;
bool button4;
bool button5;
} struct_message;

// point :)
typedef struct point {
  float x;
  float y;
} point;

bool runToEncoderTargetsWithGantry(long leftTargetTicks, long rightTargetTicks, long gantryTargetTicks, int speed);
std::array<point, 5> getCurrentOrderedCoordinateArray();
bool driveForwardsAndTrackCrack();
void driveForwardsAndUpdateCoordinates(float distance, std::vector<point>* coordinates, float gantryTarget);

// MARK: ESP-NOW Comms



// Create a struct_message called joystickData
struct_message joystickData = {1950, 1950, 1950, 1950, true, true, true};

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  // Serial.print("Bytes received: "); // Serial.println(len);
  // Serial.print("VY1 ");
  // Serial.println(joystickData.vy1);
  // Serial.print("VY2 ");
  // Serial.println(joystickData.vy2);
  // Serial.print("VY3 ");
  // Serial.println(joystickData.vy3);
  // Serial.print("VX2 ");
  // Serial.println(joystickData.vx2);
  // Serial.print("Button1: ");
  // Serial.println(joystickData.button1);
  // Serial.print("Button2: ");
  // Serial.println(joystickData.button2);
  // Serial.print("Button3: ");
  // Serial.print(joystickData.button3);
  // Serial.print("Button4: ");
  // Serial.println(joystickData.button4);
  // Serial.print("Button5: ");
  // Serial.println(joystickData.button5);
  
  // Serial.println();
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
  Serial.print("Calculation: (");
  Serial.print(distanceMm);
  Serial.print(" / ");
  Serial.print(wheelCircumference);
  Serial.print(" ) * ");
  Serial.print(ticksPerRotation);
  return lround((distanceMm / wheelCircumference) * ticksPerRotation);
}

int pidOutputToSpeed(double output, long error, int maxSpeed) {
  if (abs(error) <= encoderToleranceTicks) {
    return 0;
  }

  int pwm = abs(output);

  // Serial.print(" PWM Before Constrain: ");
  // Serial.print(pwm);

  // Serial.print(" Max Speed: ");
  // Serial.print(maxSpeed);
  // Serial.print(" PWM > Max Speed: ");
  // Serial.print(pwm > maxSpeed);
  if (pwm > maxSpeed) {
    pwm = maxSpeed;
  }
  
  // Serial.print(" PID Output: ");
  // Serial.print(output);
  // Serial.print(" PWM: ");
  // Serial.print(pwm);


  int speedFloor = min(minimumPIDSpeed, maxSpeed);
  if (pwm < speedFloor) {
    pwm = speedFloor;
  }

  return output >= 0 ? pwm : -pwm;
}

// MARK: PID Logic ish
bool runToEncoderTargets(long leftTargetTicks, long rightTargetTicks, int speed) {
  int maxSpeed = constrain(abs(speed), 0, 255);
  if (maxSpeed == 0 || (leftTargetTicks == 0 && rightTargetTicks == 0)) {
    stopAllDriveMotors();
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
    // Serial.println();

    if (leftAtTarget && rightAtTarget) {
      stopAllDriveMotors();
      return true;
    }

    if (millis() - startTime > timeoutMs) {
      stopAllDriveMotors();
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
      // Serial.print("Output Speed:");
      // Serial.println(leftSpeed);
    } else {
      setSpeed(leftSpeed, rightSpeed);
    }
    delay(movementLoopDelayMs);
  }
}

void driveDistance(float distance, int speed) {
  long targetTicks = distanceToEncoderTicks(distance);
  Serial.print(" Target Ticks: ");
  Serial.println(targetTicks);
  runToEncoderTargets(targetTicks, targetTicks, speed);
}

void turnDegrees(float degrees, int speed) {
  float wheelTravelMm = trackWidth * turnSlipCompensation * PI * (degrees / 360.0);
  long targetTicks = distanceToEncoderTicks(wheelTravelMm);
  runToEncoderTargets(targetTicks, -targetTicks, speed);
}

// MARK: Drive & Crack Functions

// Origin Coordinates
float cameraXOrigin = 0.5; // Normalized X coordinate of the camera's origin 
float cameraYOrigin = 0.5; // -2; // Normalized Y coordinate of the camera's origin
float cameraToRobotCenterMM = 87; // TODO
float gantryY_MM = -94; // TODO
float minGantryX_MM = -(185.7 / 2);
float maxGantryX_MM = maxGantryPosition / gantryTicksPerMM + minGantryX_MM;

float getGantryX() {
  noInterrupts();
  long gantryTicks = encoderValueGantry;
  interrupts();
  return gantryTicks/gantryTicksPerMM + minGantryX_MM;
}

float normalizedToRobotRelativeX(float x) {
  float x_mm = (x - 0.5) * cameraFOVWidthMM;
  return x_mm;
}

float normalizedToRobotRelativeY(float y) {
  float y_mm = (0.5 - y) * cameraFOVHeightMM;
  return y_mm;
}

// Calculating distance to the center of a crack based on normalized coordinates (cx, cy) of the crack in the camera's field of view
float distanceToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - cameraXOrigin) * cameraFOVWidthMM;
  float y_mm = abs((cameraYOrigin - cy)) * cameraFOVHeightMM;

  Serial.println("X_mm: " + String(x_mm));
  Serial.println("Y_mm: " + String(y_mm));

  // Calculate distance to the center of the crack using Pythagorean theorem
  return sqrt(x_mm * x_mm + y_mm * y_mm);
}

float angleToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - cameraXOrigin) * cameraFOVWidthMM;
  float y_mm = (cy - cameraYOrigin) * cameraFOVHeightMM;

  // Calculate angle to the center of the crack using arctangent
  return atan2(x_mm, -y_mm) * (180 / PI); // atan2 is flipped so that 0 degrees is forward and positive angles are to the right
}

 bool isDriveAligned(){
  float distance = distanceToCrackCenter(cx, cy);
  float angle = angleToCrackCenter(cx, cy);

  // Define thresholds for alignment
  const float distanceThreshold = 1; // mm
  const float angleThreshold = 5.0; // degrees

  // Check if the robot is aligned with the crack center
  return (abs(distance) < distanceThreshold) && (abs(angle) < angleThreshold);
 }

void gantryAlign(float cx){
  
  // Convert normalized pixel coordinates to millimeters
  float x_mm = ((cx - 0.5) * cameraFOVWidthMM) + 92;

  // Calculate the distance to move the gantry based on the x_mm value
  float distanceToMove = x_mm; // Assuming 1:1 mapping for simplicity, adjust as necessary
  Serial.print("Distance To Move");
  Serial.println(distanceToMove);

  // Move the gantry to align with the crack center
  setGantryPosition(distanceToMove, 100);
}

// Drives to the crack's center such that the crack is centered in the camera's field of view at (0.5, 0.5) 
void driveToCrackCenter(float cx, float cy) {
  float distance = distanceToCrackCenter(cx, cy);
  float angle = angleToCrackCenter(cx, cy);

  if (angle > 90) {
    angle -= 180;
    // distance = -distance;
  } else if (angle < -90) {
    angle += 180;
    // distance = -distance;
  }

  Serial.print("Driving to crack center: distance = ");
  Serial.print(distance);
  Serial.print(" mm, angle = ");
  Serial.print(angle);
  Serial.println(" degrees");

  // Rotate to face the crack center
  turnDegrees(angle, turnSpeed);
  
  // Drive forward to the crack center
  driveDistance(distance, movementSpeed);

  // Rotate back to forwards
  turnDegrees(-angle, turnSpeed);
}

void driveToCrackCenterNoRot(float cx, float cy){
  float distance = distanceToCrackCenter(cx, cy);
  angle = angleToCrackCenter(cx, cy) - angle;

  if (angle > 90) {
    angle -= 180;
    distance = -distance;
  } else if (angle < -90) {
    angle += 180;
    distance = -distance;
  }

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

void driveToCoordinate(float x, float y) {
  float distance = sqrt(x*x+y*y);
  float angle = atan2(x, y) * (180 / PI);;

  if (angle > 90) {
    angle -= 180;
    distance = -distance;
  } else if (angle < -90) {
    angle += 180;
    distance = -distance;
  }

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

void getPointAToPointB(float ax, float ay, float bx, float by) {
  
}

void getExtruderToPoint(float x, float y) {
  // Checking if the point is in range of the gantry

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
  digitalWrite(L1, HIGH);
  digitalWrite(L2, HIGH);
  digitalWrite(PWML, 0);
}

void stopRightMotors() {
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);
  digitalWrite(PWMR, 0);
}

void stopAllDriveMotors() {
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

// MARK: Gantry Functions

void gantryDirectionForward() {
  digitalWrite(Gantry1, HIGH);
  digitalWrite(Gantry2, LOW);
}

void gantryDirectionBackward() {
  digitalWrite(Gantry1, LOW);
  digitalWrite(Gantry2, HIGH);
}

void stopGantry() {
  digitalWrite(Gantry1, HIGH);
  digitalWrite(Gantry2, HIGH);
  analogWrite(ENI, 0);
}

void setGantryPower(int power) {
  long pos = readEncoderGantry();
  if (hasZeroedGantry) {
    if (power < 0 && pos <= minGantryPosition) {
      Serial.print(" Gantry Min Hit ");
      power = 0;
    }
    if (power > 0 && pos >= maxGantryPosition) power = 0;
  }

  if (power == 0) {
    stopGantry();
  } else if (power < 0) {
    power = -power;
    gantryDirectionBackward();
  } else {
    gantryDirectionForward();
  
  }
  if (power > 255) {
    power = 255;
  }
  analogWrite(ENI, power);
}

long readEncoderGantry() {
  noInterrupts();
  long value = encoderValueGantry;
  interrupts();
  return value;
}

bool setGantryPosition(int targetPosMM, int maxSpeed) {
  int targetPos = (targetPosMM - minGantryX_MM) * gantryTicksPerMM;
  targetPos = constrain(targetPos, minGantryPosition, maxGantryPosition);
  long startLeft = readEncoderGantry();

  pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);

  while (true) {
    long current = readEncoderGantry();
    long error = targetPos - current;

    bool atTarget = abs(error) <= encoderToleranceTicks;

    // Serial.print("Gantry Error: ");
    // Serial.print(error);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(", Target: ");
    Serial.print(targetPos);   
    Serial.print(", Current: ");
    Serial.print(current);
    // Serial.println();
    if (atTarget) {
      stopGantry();
      return true;
    }
  
    pidControllerGantry.UpdateError(error);
    double output = pidControllerGantry.TotalError();
    Serial.print(" Gantry PID Output: ");
    Serial.print(output);
    Serial.print(", P: ");
    Serial.print(pidControllerGantry.p_error);
    Serial.print(", I: ");
    Serial.print(pidControllerGantry.i_error);
    Serial.print(", D: ");
    Serial.print(pidControllerGantry.d_error);

    int speed = atTarget ? 0 : pidOutputToSpeed(output, error, maxSpeed);
    setGantryPower(speed);
    Serial.print(" Gantry Speed: ");
    Serial.println(speed);
    delay(movementLoopDelayMs);
  }
}

bool isAtTargetPositionGantry() {
  return abs(targetPositionGantry - encoderValueGantry) < 5;
}

bool homeGantryPosition() {
  int previousGantryPos = readEncoderGantry();
  setGantryPower(100); // Move gantry backward slowly
  delay(1000);
  int currentPosition = readEncoderGantry();
  while (true) {
    if (currentPosition == previousGantryPos){
      Serial.println("Gantry homed at position: " + String(currentPosition));
      stopGantry();
      noInterrupts();
      encoderValueGantry = 0;
      interrupts();
      hasZeroedGantry = true;
      return true;
    }
    previousGantryPos = currentPosition;
    delay(1000);
    currentPosition = readEncoderGantry();
  }
  return false;
  // while (currentPos)
}

// MARK: Extruder Functions

void extDirectionForward() {
  digitalWrite(EXT1, HIGH);
  digitalWrite(EXT2, LOW);
}

void extDirectionBackward() {
  digitalWrite(EXT1, LOW);
  digitalWrite(EXT2, HIGH);
}

void stopExtruder() {
  digitalWrite(EXT1, LOW);
  digitalWrite(EXT2, LOW);
  analogWrite(PWM_EXT, 0);
}

void setExtruderPower(int power) {
  if (power == 0) {
    stopExtruder();
  } else if (power < 0) {
    power = -power;
    extDirectionBackward();
  } else {
    extDirectionForward();
  }
  analogWrite(PWM_EXT, power);
}

bool setExtruderPosition(int targetPosMM, int maxSpeed){
  int targetPos = targetPosMM * gantryTicksPerMM;
  targetPos = constrain(targetPos, minGantryPosition, maxGantryPosition);
  long startLeft = readEncoderExtruder();

  pidControllerExt.Init(kP_ext, kI_ext, kD_ext);

  while (true) {
    long current = readEncoderExtruder();
    long error = targetPos - current;

    bool atTarget = abs(error) <= encoderToleranceTicks;

    Serial.print("Extruder Error: ");
    Serial.print(error);
    Serial.print(", Target: ");
    Serial.print(targetPos);   
    Serial.print(", Current: ");
    Serial.print(current);

    if (atTarget) {
      stopGantry();
      return true;
    }
    
    pidControllerExt.UpdateError(error);
    double output = pidControllerExt.TotalError();
    Serial.print(" Extruder PID Output: ");
    Serial.print(output);



    int speed = atTarget ? 0 : pidOutputToSpeed(output, error, maxSpeed);
    setExtruderPower(speed);
    Serial.print(" Extruder Speed: ");
    Serial.println(speed);
    delay(movementLoopDelayMs);
  }
}

// MARK: Encoder Updates

void updateEncoderLeft(){
  if (digitalRead(ENCAFL) > digitalRead(ENCBFL))
    encoderValueLeft++;
  else
    encoderValueLeft--;
}

void updateEncoderRight(){
  if (digitalRead(ENCAFR) > digitalRead(ENCBFR))
    encoderValueRight--;
  else
    encoderValueRight++;
}

void updateEncoderGantry(){
  if (digitalRead(ENCGANA) > digitalRead(ENCGANB))
    encoderValueGantry--;
  else
    encoderValueGantry++;
}

void updateEncoderExtruder(){
  if(digitalRead(ENCEXTA) > digitalRead(ENCEXTB)){
    encoderValueExtruder++;
  } else{
    encoderValueExtruder--;
  }
}

int readEncoderExtruder(){
  return encoderValueExtruder;
}

bool followPath(float ax[], float ay[]){
  bool proceed = false;
  for(int i = 4; i >= 0; i--){
    if(ax[i] == -1 && ay[i] == -1){
      Serial.println("No crack Detected");
      return false;
    }

    if(i != 4){
      cameraYOrigin = 0.5;
      float aax[5];
      float aay[5];
      aax[i] = ax[i] + (0.5-ax[i+1]);
      aay[i] = ay[i] + (0.5-ay[i+1]);
      Serial.println("AX: " + String(aax[i]));
      Serial.println("AY: " + String(aay[i]));
      driveToCrackCenterNoRot(aax[i], aay[i]);
    } else{
    driveToCrackCenterNoRot(ax[i], ay[i]);
    }
    // gantryAlign(ax[i]);
  }
  cameraYOrigin = -2;
  return true;
}

bool crackAuto() {
  float dCX = cx;
  float dCY = cy;
  driveToCrackCenter(dCX, dCY);
  gantryAlign(dCX);
  return true;
}

bool testAutoDrive() {
    int dCX = cx;
      int dCY = cy;
      while(true){
        driveToCrackCenter(dCX, dCY);
      }
      Serial.println("Aligned");
    return true;
}

void updateVision(){
  if (BS.available() > 0) {
    Serial.println("Connected");
    String rawData = BS.readStringUntil('\n');
    rawData.trim();
    Serial.println(rawData);
    if (rawData.length() > 0) {
      int cxIndex = rawData.indexOf("cx");
      int cyIndex = rawData.indexOf("cy");
      int xIndex = rawData.indexOf("x1");
      String xData = rawData.substring(cxIndex + 4, xIndex - cxIndex - 3);
      String yData = rawData.substring(cyIndex + 4, xIndex - cyIndex - 3);
      cx = xData.toFloat();
      cy = yData.toFloat();
      Serial.println("x" + xData);
      Serial.println("y" + yData);
      if(cx == -1 && cy == -1){
        cx = 0.5;
        cy = 0.5;
      }
    // Serial.println(rawData);
    }
  }
}

void updateVisionArray(){
  if (BS.available() > 0){
    String rawData = BS.readStringUntil('\n');
    rawData.trim();
    if(rawData.length() > 0){
      int x1Index = rawData.indexOf("x1");
      int x2Index = rawData.indexOf("x2");
      int x3Index = rawData.indexOf("x3");
      int x4Index = rawData.indexOf("x4");
      int cxIndex = rawData.indexOf("cx");

      int y1Index = rawData.indexOf("y1");
      int y2Index = rawData.indexOf("y2");
      int y3Index = rawData.indexOf("y3");
      int y4Index = rawData.indexOf("y4");
      int cyIndex = rawData.indexOf("cy");
      
      String x1Data = rawData.substring(x1Index + 4, x2Index - x1Index - 3);
      String x2Data = rawData.substring(x2Index + 4, x3Index - x2Index - 3);
      String x3Data = rawData.substring(x3Index + 4, x4Index - x3Index - 3);
      String x4Data = rawData.substring(x4Index + 4, cxIndex - x4Index - 3);
      String cxData = rawData.substring(cxIndex + 4, cyIndex - cxIndex - 3);

      String y1Data = rawData.substring(cyIndex + 4, cxIndex - cyIndex - 3);
      String y2Data = rawData.substring(y1Index + 4, y2Index - y1Index - 3);
      String y3Data = rawData.substring(y2Index + 4, y3Index - y2Index - 3);
      String y4Data = rawData.substring(y3Index + 4, y4Index - y3Index - 3);
      String cyData = rawData.substring(y4Index + 4, y3Index - y4Index - 3);
      
      ax[0] = cxData.toFloat();
      ax[1] = x1Data.toFloat();
      ax[2] = x2Data.toFloat();
      ax[3] = x3Data.toFloat();
      ax[4] = x4Data.toFloat();

      ay[0] = y1Data.toFloat();
      ay[1] = y2Data.toFloat();
      ay[2] = y3Data.toFloat();
      ay[3]= y4Data.toFloat();
      ay[4] = y4Data.toFloat();
    }
  }
}

// MARK: Setup
void setup() {

  Serial.begin(115200);
  BS.begin("ARCS");

  // Set all control pins to outputs
  pinMode(PWML, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(Gantry1, OUTPUT);
  pinMode(Gantry2, OUTPUT);
  pinMode(ENI, OUTPUT);
  pinMode(PWM_EXT, OUTPUT);
  pinMode(EXT1, OUTPUT);
  pinMode(EXT2, OUTPUT);

  // Set all encoder pins to inputs
  pinMode(ENCAFL, INPUT);
  pinMode(ENCBFL, INPUT);
  pinMode(ENCAFR, INPUT);
  pinMode(ENCBFR, INPUT);
  pinMode(ENCGANA, INPUT);
  pinMode(ENCGANB, INPUT);
  pinMode(ENCEXTA, INPUT);
  pinMode(ENCEXTB, INPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCGANA), updateEncoderGantry, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCEXTA), updateEncoderExtruder, RISING);
  
  servo.attach(servoPin);
  servo.writeMicroseconds(1100); // send "stop" signal to ESC. Also necessary to arm the ESC.
  Serial.println("ESC TEST PREP");
  
  // // Do we even need these inits?
  // pidController.Init(kP, kI, kD);
  // pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);
  // leftDrivePID.Init(kP, kI, kD);
  // rightDrivePID.Init(kP, kI, kD);
  // pidControllerExt.Init(kP_ext, kI_ext, kD_ext);

  // Turn off motors initially
  stopAllDriveMotors();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
}

// MARK: Loop
long encoderStartLeft = 0;
long encoderStartRight = 0;
long encoderEndLeft = 0;
long encoderEndRight = 0;
float rpmLeft = 0.0;
float rpmRight = 0.0;
// int pwmValue = 255;

const int edfPowerMin = 1100;
const int edfPowerMax = 1900;
const int edfJoystickDefault = 1950;
int edfPower = edfPowerMin;
int edfIncrementResolution = 10;
int edfIncrement = 0;
int joystickDeadzone = 50;
bool killEverything = false;
bool joystickConnected = true;

void loop() {
  // Serial.println(readEncoderExtruder());
  // setExtruderPower(100);
  setExtruderPosition(100, 255);
  while(true){}
  // while(true){}
  // setGantryPower(100);
  // updateVision();
  // // // updateVisionArray();
  // // Serial.println("CX: " +String(cx));
  // // Serial.println("CY: " + String(cy));

  // if(BS.available() > 0){
  //   Serial.println("Starting Auto");
  //   crackAuto();
  // }
  // Serial.println(ax);

  // delay(100);
  // turnDegrees(180, 255);
  // driveDistance(45*PI, 255);
  // forwards();
  // homeGantryPosition();
  // driveToCrackCenter(0.5, 0.5);
  // gantryAlign(0.5);
  // crackAuto();
  // while (true){}
  // forwards();
  // turnDegrees(90, 200);
  // while (true) {}

  // setGantryPosition(maxGantryPosition/2, 255);
  // crackAuto();
  // while (true) {}
  // Serial.println("Gantry Position: " + String(encoderValueGantry));

  // setGantryPower(0);
  // delay(1000);

  // testAutoDrive();

  // MARK: Joystick Code
  
  
  int leftVal=map(joystickData.vy1, 0, 4095, -255, 255);
  int rightVal=map(joystickData.vy3, 0, 4095, -255, 255);
  int gantryVal=map(joystickData.vx2, 0, 4095, -255, 255);
  int extVal = map(joystickData.vy2, 0, 4095, -255, 255);
  
  if (abs(leftVal) < joystickDeadzone) {
    leftVal = 0;
  }
  if (abs(rightVal) < joystickDeadzone) {
    rightVal = 0;
  }
  if (abs(gantryVal) < joystickDeadzone) {
    gantryVal = 0;
  }
  if (abs(extVal) < joystickDeadzone) {
    extVal = 0;
  }
  bool button3State=joystickData.button3;
  bool button2State=joystickData.button2;
  bool button1State=joystickData.button1;
  bool button4State=joystickData.button4;
  bool button5State=joystickData.button5;

  edfIncrement = 0;
  if (joystickData.vy3 < edfJoystickDefault) {
    edfIncrement=map(joystickData.vy2, 0, edfJoystickDefault-200, -edfIncrementResolution, 0);
  } else {
    edfIncrement=map(joystickData.vy2, edfJoystickDefault+100, 4095, 0, edfIncrementResolution);
  }

  gantryVal = -gantryVal;

  edfIncrement = -edfIncrement;
  edfIncrement /= edfIncrementResolution; // Normalize to -1 to 1
  edfIncrement *= 3;
  if (button1State == 0) {
    killEverything = false;
    // hasZeroedGantry = true;
    autonomous = false;
  } else if (button3State == 0) {
    killEverything = true;
  }
  if (killEverything || !joystickConnected) {
    edfPower = edfPowerMin;
    servo.writeMicroseconds(edfPower); // output to edfs
    stopAllDriveMotors();
    analogWrite(ENI, 0);
    setExtruderPower(0);
  } else {
    if (button2State == 0) {
      edfPower += edfIncrement;
      edfPower = constrain(edfPower, edfPowerMin, edfPowerMax);
      servo.writeMicroseconds(edfPower); // output to edfs
    } 

    // Serial.print("Setting Speed");
    setSpeed(leftVal, rightVal);
    setGantryPower(gantryVal);
    setExtruderPower(extVal);
  }
    
  
  Serial.print(" EDF Increment: ");
  Serial.print(edfIncrement);
  Serial.print(" EDF Power: ");
  Serial.print(edfPower);
  Serial.print(" Left Val: ");
  Serial.print(leftVal);
  Serial.print(" Right Val: ");
  Serial.print(rightVal);
  Serial.print(" Gantry Val: ");
  Serial.print(gantryVal);
  Serial.print(" Kill Everything: ");
  Serial.print(killEverything);
  Serial.println();
  
  // Serial.println(encoderValueLeft + " hello " + encoderValueRight); // LINE OF DOOOOOOOOOOOOOOOOOOOMMMMMMMMMMM ONLY UNCOMMENT IN CASE OF EMERGENCY

  // driveDistance(700, 255);

  // Serial.print(String(encoderValueLeft));
  // Serial.print(", ");
  // Serial.print(String(encoderValueRight));
  // Serial.print(", ");
  // Serial.println(String(encoderValueGantry));

  // digitalWrite(Gantry1, HIGH);
  // digitalWrite(Gantry2, LOW);
  // analogWrite(ENI, 100);

//   if (BS.available() > 0) {
//     String rawData = Serial.readStringUntil('\n');
//     rawData.trim();
//     if (rawData.length() > 0) {
//       int cxIndex = rawData.indexOf("cx");
//       int cyIndex = rawData.indexOf("cy");
//       int xIndex = rawData.indexOf("x1");
//       String xData = rawData.substring(cxIndex + 4, cyIndex - cxIndex - 3);
//       String yData = rawData.substring(cyIndex + 4, xIndex - cyIndex - 3);
//       cx = xData.toFloat();
//       cy = yData.toFloat();
//     Serial.println(rawData);
//     }
//   }
// //   
// if(button4State == LOW && prevButton4State == HIGH){
// autonomous = true;
// followPath(ax, ay);
// autonomous = false;
// button4State = prevButton4State;
// }

// if(button5State == LOW && prevButton5State == HIGH){
// gantryAlign(cx);
// button5State = prevButton5State;
// }
  // Serial.print(String((ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));
}

// MARK: New Tracking Code

const float invalidTrackingCoordinateMM = -1000000.0;

bool runToEncoderTargetsWithGantry(long leftTargetTicks, long rightTargetTicks, long gantryTargetTicks, int speed) {
  int maxSpeed = constrain(abs(speed), 0, 255);
  if (maxSpeed == 0 || (leftTargetTicks == 0 && rightTargetTicks == 0 && gantryTargetTicks == 0)) {
    stopAllDriveMotors();
    stopGantry();
    return true;
  }

  long startLeft = readEncoderLeft();
  long startRight = readEncoderRight();
  long startGantry = readEncoderGantry();
  targetPositionLeft = startLeft + leftTargetTicks;
  targetPositionRight = startRight + rightTargetTicks;
  targetPositionGantry = startGantry + gantryTargetTicks;

  Serial.print("[TRACK MOVE] leftTicks=");
  Serial.print(leftTargetTicks);
  Serial.print(" rightTicks=");
  Serial.print(rightTargetTicks);
  Serial.print(" gantryTicks=");
  Serial.print(gantryTargetTicks);
  Serial.print(" maxSpeed=");
  Serial.println(maxSpeed);

  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);
  pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);

  float longestDriveDistanceMm = (max(abs(leftTargetTicks), abs(rightTargetTicks)) / (float)ticksPerRotation) * wheelCircumference;
  float gantryDistanceMm = abs(gantryTargetTicks) / gantryTicksPerMM;
  float longestDistanceMm = max(longestDriveDistanceMm, gantryDistanceMm);
  unsigned long timeoutMs = (unsigned long)((longestDistanceMm / mmPerSecond(maxSpeed)) * 3000.0) + 1000;
  unsigned long startTime = millis();

  while (true) {
    long currentLeft = readEncoderLeft();
    long currentRight = readEncoderRight();
    long currentGantry = readEncoderGantry();
    long leftError = targetPositionLeft - currentLeft;
    long rightError = targetPositionRight - currentRight;
    long gantryError = targetPositionGantry - currentGantry;

    bool leftAtTarget = abs(leftError) <= encoderToleranceTicks;
    bool rightAtTarget = abs(rightError) <= encoderToleranceTicks;
    bool gantryAtTarget = abs(gantryError) <= encoderToleranceTicks;

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

    if (leftAtTarget && rightAtTarget && gantryAtTarget) {
      stopAllDriveMotors();
      stopGantry();
      Serial.print("[TRACK MOVE DONE] left=");
      Serial.print(currentLeft);
      Serial.print(" right=");
      Serial.print(currentRight);
      Serial.print(" gantry=");
      Serial.println(currentGantry);
      return true;
    }

    if (millis() - startTime > timeoutMs) {
      stopAllDriveMotors();
      stopGantry();
      Serial.print("[TRACK MOVE TIMEOUT] leftError=");
      Serial.print(leftError);
      Serial.print(" rightError=");
      Serial.print(rightError);
      Serial.print(" gantryError=");
      Serial.println(gantryError);
      return false;
    }

    leftDrivePID.UpdateError(leftError);
    rightDrivePID.UpdateError(rightError);
    pidControllerGantry.UpdateError(gantryError);
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
    double gantryOutput = pidControllerGantry.TotalError();

    // Serial.print(", Left Output: ");
    // Serial.print(leftOutput);
    // Serial.print(", Right Output: ");
    // Serial.println(rightOutput);

    int leftSpeed = leftAtTarget ? 0 : pidOutputToSpeed(leftOutput, leftError, maxSpeed);
    int rightSpeed = rightAtTarget ? 0 : pidOutputToSpeed(rightOutput, rightError, maxSpeed);
    int gantrySpeed = gantryAtTarget ? 0 : pidOutputToSpeed(gantryOutput, gantryError, maxSpeed);

    // Serial.print("[TRACK MOVE LOOP] leftError=");
    // Serial.print(leftError);
    // Serial.print(" rightError=");
    // Serial.print(rightError);
    // Serial.print(" gantryError=");
    // Serial.print(gantryError);
    // Serial.print(" leftSpeed=");
    // Serial.print(leftSpeed);
    // Serial.print(" rightSpeed=");
    // Serial.print(rightSpeed);
    // Serial.print(" gantrySpeed=");
    // Serial.println(gantrySpeed);

    if (leftTargetTicks == -rightTargetTicks) {
      // If turning, ensure both motors are moving at the same speed
      setSpeed(leftSpeed, -leftSpeed);
      // Serial.print("Output Speed:");
      // Serial.println(leftSpeed);
    } else {
      setSpeed(leftSpeed, rightSpeed);
    }
    setGantryPower(gantrySpeed);
    delay(movementLoopDelayMs);
  }
}

float parseVisionCoordinate(String rawData, String key) {
  int keyIndex = rawData.indexOf(key);
  if (keyIndex < 0) {
    return -1.0;
  }

  int valueStart = keyIndex + key.length();
  while (valueStart < rawData.length()) {
    char currentChar = rawData.charAt(valueStart);
    if (currentChar == '-' || currentChar == '.' || (currentChar >= '0' && currentChar <= '9')) {
      break;
    }
    valueStart++;
  }

  int valueEnd = valueStart;
  if (valueEnd < rawData.length() && rawData.charAt(valueEnd) == '-') {
    valueEnd++;
  }
  while (valueEnd < rawData.length()) {
    char currentChar = rawData.charAt(valueEnd);
    if (currentChar != '.' && (currentChar < '0' || currentChar > '9')) {
      break;
    }
    valueEnd++;
  }

  if (valueStart >= rawData.length() || valueEnd <= valueStart) {
    return -1.0;
  }
  return rawData.substring(valueStart, valueEnd).toFloat();
}

point normalizedToRobotRelativePoint(float x, float y) {
  if (x < 0.0 || y < 0.0) {
    return {invalidTrackingCoordinateMM, invalidTrackingCoordinateMM};
  }

  return {
    normalizedToRobotRelativeX(x),
    normalizedToRobotRelativeY(y) + cameraToRobotCenterMM
  };
}

bool isDetectedPoint(point coordinate) {
  return coordinate.x > invalidTrackingCoordinateMM / 2.0 && coordinate.y > invalidTrackingCoordinateMM / 2.0;
}

bool isDuplicateTrackedPoint(std::vector<point>* coordinates, point candidate) {
  const float duplicateToleranceMM = 5.0;
  for (int i = 0; i < (int)coordinates->size(); i++) {
    point existing = coordinates->at(i);
    if (!isDetectedPoint(existing)) {
      continue;
    }
    float xDifference = existing.x > candidate.x ? existing.x - candidate.x : candidate.x - existing.x;
    float yDifference = existing.y > candidate.y ? existing.y - candidate.y : candidate.y - existing.y;
    if (xDifference <= duplicateToleranceMM && yDifference <= duplicateToleranceMM) {
      return true;
    }
  }
  return false;
}

std::array<point, 5> getCurrentOrderedCoordinateArray() {
  // It should order it in the order y4, y3, cy, y2, y1
  // AKA x4, x3, cx, x2, x1
  // It should take the values directly from the bluetooth, i think the updateVisionArray does this but it shouldn't use that function
  // the bluetooth gives normalized coordinates relative to the camera FOV with 0,0 being the top left and 1,1 being the bottom right
  // if the normalized coordinate says -1 then it isn't detecting a crack
  // you have to convert the coordinates to robot relative and in MM, there is already logic that does this like float normalizedToRobotRelativeX(float x) and functions around that area
  std::array<point, 5> points = {{{invalidTrackingCoordinateMM, invalidTrackingCoordinateMM}, {invalidTrackingCoordinateMM, invalidTrackingCoordinateMM}, {invalidTrackingCoordinateMM, invalidTrackingCoordinateMM}, {invalidTrackingCoordinateMM, invalidTrackingCoordinateMM}, {invalidTrackingCoordinateMM, invalidTrackingCoordinateMM}}};
  if (BS.available() <= 0) {
    return points;
  }

  String rawData = BS.readStringUntil('\n');
  rawData.trim();
  if (rawData.length() == 0) {
    return points;
  }

  points[0] = normalizedToRobotRelativePoint(parseVisionCoordinate(rawData, "x4"), parseVisionCoordinate(rawData, "y4"));
  points[1] = normalizedToRobotRelativePoint(parseVisionCoordinate(rawData, "x3"), parseVisionCoordinate(rawData, "y3"));
  points[2] = normalizedToRobotRelativePoint(parseVisionCoordinate(rawData, "cx"), parseVisionCoordinate(rawData, "cy"));
  points[3] = normalizedToRobotRelativePoint(parseVisionCoordinate(rawData, "x2"), parseVisionCoordinate(rawData, "y2"));
  points[4] = normalizedToRobotRelativePoint(parseVisionCoordinate(rawData, "x1"), parseVisionCoordinate(rawData, "y1"));

  Serial.print("[TRACK VISION RAW] ");
  Serial.println(rawData);
  // for (int i = 0; i < (int)points.size(); i++) {
  //   Serial.print("[TRACK VISION POINT] index=");
  //   Serial.print(i);
  //   Serial.print(" x=");
  //   Serial.print(points[i].x);
  //   Serial.print(" y=");
  //   Serial.println(points[i].y);
  // }
  return points;
}

// This function will drive the robot forwards once the robot is already aligned with the crack
// it will stop once the crack is no longer detected AND the robot has fully filled the last detected crack
bool driveForwardsAndTrackCrack() {
  std::vector<point> coordinates;
  const int numberOfLoops = 40;
  const float filledPointToleranceMM = 8.0;
  bool hasSeenCrack = false;

  Serial.println("[TRACK] start");

  for (int i = 0; i < numberOfLoops; i++) {
    std::array<point, 5> currentPoints = getCurrentOrderedCoordinateArray();
    bool detectedCrackThisLoop = false;
    int newPointsThisLoop = 0;

    for (int j = 0; j < (int)currentPoints.size(); j++) {
      point currentPoint = currentPoints[j];
      if (!isDetectedPoint(currentPoint)) {
        continue;
      }
      detectedCrackThisLoop = true;
      hasSeenCrack = true;
      if (!isDuplicateTrackedPoint(&coordinates, currentPoint)) {
        coordinates.push_back(currentPoint);
        newPointsThisLoop++;
      }
    }

    Serial.print("[TRACK] loop=");
    Serial.print(i);
    Serial.print(" detected=");
    Serial.print(detectedCrackThisLoop);
    Serial.print(" newPoints=");
    Serial.print(newPointsThisLoop);
    Serial.print(" queued=");
    Serial.println((int)coordinates.size());
    // for (int j = 0; j < (int)coordinates.size(); j++) {
    //   Serial.print("[TRACK QUEUE] index=");
    //   Serial.print(j);
    //   Serial.print(" x=");
    //   Serial.print(coordinates[j].x);
    //   Serial.print(" y=");
    //   Serial.println(coordinates[j].y);
    // }

    int targetIndex = -1;
    float shortestDistanceToGantry = 1000000.0;
    for (int j = 0; j < (int)coordinates.size(); j++) {
      float distanceToGantry = coordinates[j].y - gantryY_MM;
      if (distanceToGantry >= -filledPointToleranceMM && distanceToGantry < shortestDistanceToGantry) {
        shortestDistanceToGantry = max(0.0f, distanceToGantry);
        targetIndex = j;
      }
    }

    if (targetIndex < 0) {
      for (int j = (int)coordinates.size() - 1; j >= 0; j--) {
        if (coordinates[j].y <= gantryY_MM + filledPointToleranceMM) {
          coordinates.erase(coordinates.begin() + j);
        }
      }
      if (!detectedCrackThisLoop && coordinates.empty()) {
        stopAllDriveMotors();
        stopGantry();
        Serial.print("[TRACK] done; hasSeenCrack=");
        Serial.println(hasSeenCrack);
        return hasSeenCrack;
      }
      Serial.println("[TRACK] no usable target yet");
      delay(20);
      continue;
    }

    Serial.print("[TRACK] targetIndex=");
    Serial.print(targetIndex);
    Serial.print(" targetX=");
    Serial.print(coordinates[targetIndex].x);
    Serial.print(" targetY=");
    Serial.print(coordinates[targetIndex].y);
    Serial.print(" distanceToGantry=");
    Serial.println(shortestDistanceToGantry);

    if (shortestDistanceToGantry > cameraFOVHeightMM) {
      Serial.print("[TRACK] strideForwardMM=");
      Serial.println(cameraFOVHeightMM);
      driveForwardsAndUpdateCoordinates(cameraFOVHeightMM, &coordinates, invalidTrackingCoordinateMM);
    } else {
      Serial.print("[TRACK] moveToPointMM=");
      Serial.print(shortestDistanceToGantry);
      Serial.print(" gantryTargetX=");
      Serial.println(coordinates[targetIndex].x);
      driveForwardsAndUpdateCoordinates(shortestDistanceToGantry, &coordinates, coordinates[targetIndex].x);
    }

    for (int j = (int)coordinates.size() - 1; j >= 0; j--) {
      if (coordinates[j].y <= gantryY_MM + filledPointToleranceMM) {
        coordinates.erase(coordinates.begin() + j);
      }
    }

    if (!detectedCrackThisLoop && coordinates.empty()) {
      stopAllDriveMotors();
      stopGantry();
      Serial.println("[TRACK] done; all queued points filled");
      return true;
    }
  }

  stopAllDriveMotors();
  stopGantry();
  Serial.println("[TRACK] stopped after max loops");
  return false;
}

// This function drives forwards a certain distance and then updates the coordinates so that the y value reduces by the amount the robot moved forwards
// if gantry target has a value then it will also move the gantry to its target position
// the gantryTarget variable will be given as a robot relative x value in MM
// use the runToEncoderTargetsWithGantry() function
void driveForwardsAndUpdateCoordinates(float distance, std::vector<point>* coordinates, float gantryTarget) {
  // float maxGantryX_MM = -minGantryX_MM;
  long gantryTargetTicks = 0;
  bool movingGantry = false;
  if (gantryTarget >= minGantryX_MM && gantryTarget <= maxGantryX_MM) {
    long absoluteGantryTargetTicks = lround((gantryTarget - minGantryX_MM) * gantryTicksPerMM);
    gantryTargetTicks = absoluteGantryTargetTicks - readEncoderGantry();
    movingGantry = true;
  }

  long driveTargetTicks = distanceToEncoderTicks(distance);
  Serial.print("[TRACK UPDATE] distanceMM=");
  Serial.print(distance);
  Serial.print(" driveTicks=");
  Serial.print(driveTargetTicks);
  Serial.print(" movingGantry=");
  Serial.print(movingGantry);
  if (movingGantry) {
    Serial.print(" gantryTargetX=");
    Serial.print(gantryTarget);
    Serial.print(" gantryTicks=");
    Serial.print(gantryTargetTicks);
  }
  Serial.println();

  if (!runToEncoderTargetsWithGantry(driveTargetTicks, driveTargetTicks, gantryTargetTicks, movementSpeed)) {
    Serial.println("[TRACK UPDATE] move failed; coordinates not updated");
    return;
  }

  for (int i = 0; i < (int)coordinates->size(); i++) {
    coordinates->at(i).y -= distance;
  }
  Serial.print("[TRACK UPDATE] coordinates advanced; queued=");
  Serial.println((int)coordinates->size());
}
