#include <Arduino.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <BluetoothSerial.h>
#include <PID.h>
#include <esp_now.h>
#include <WiFi.h>

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

// MARK: Pinout

// Left Motor Pins
const int PWML = 23; 
const int L1 = 21; // const int L1 = 22;
const int L2 = 22; // const int L2 = 21;

// Right Motor Pins
const int PWMR = 19; 
const int R1 = 18;
const int R2 = 5;

// Encoder Connections
const int ENCAFL = 34; // Encoder A pin for Front Left Motor
const int ENCBFL = 35; // Encoder B pin for Front Left Motor

const int ENCAFR = 32; // Encoder A pin for Front Right Motor
const int ENCBFR = 33; // Encoder B pin for Front Right Motor

const int ENCGANA = 36; // 1st Encoder Pin for Gantry
const int ENCGANB = 39; // 2nd Encoder Pin for Gantry 

const int ENCEXTA = 2;
const int ENCEXTB = 15;



// Gantry Motor Pins
const int ENI = 25;
const int Gantry1 = 27;
const int Gantry2 = 26; 

// Extruder Motor Pins
const int PWM_EXT = 14;
const int EXT1 = 12;
const int EXT2 = 4;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

// MARK: Variables

PID pidController = PID();
PID leftDrivePID = PID();
PID rightDrivePID = PID();
PID pidControllerGantry = PID();

BluetoothSerial BS;

// Drivetrain PID values
double kP = 0.45;
double kI = 0.0035; 
double kD = 0.0;

double kP_gantry = 0.1;
double kI_gantry = 0;
double kD_gantry = 0;
int cx = 0.85;
int cy = 0.2;

bool autonomous = false;


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
const float turnSlipCompensation = 0.8; // This is a fudge factor to account for the fact that the robot doesn't turn perfectly in place
const float trackWidth = 350.0; // mm
int movementSpeed = 255; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 200; // Speed for turning left/right (0-255) - TODO
const int encoderToleranceTicks = 15;
const int minimumPIDSpeed = 55;
const unsigned long movementLoopDelayMs = 10;

const float cameraFOVWidthMM = 95.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 70.0; // Height of the camera's field of view in millimeters - TODO

int pwrGantry = 100;
int pwrExt = 100;


// MARK: ESP-NOW Comms

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
} struct_message;

// Create a struct_message called joystickData
struct_message joystickData = {1950, 1950, 1950, 1950, true, true, true};

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
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
  // Serial.println(joystickData.button3);
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

void turnDegrees(float degrees, int speed) {
  float wheelTravelMm = trackWidth * turnSlipCompensation * PI * (degrees / 360.0);
  long targetTicks = distanceToEncoderTicks(wheelTravelMm);
  runToEncoderTargets(targetTicks, -targetTicks, speed);
}

// MARK: Drive & Crack Functions

// Origin Coordinates
float cameraXOrigin = 0.5; // Normalized X coordinate of the camera's origin 
float cameraYOrigin =-2; // Normalized Y coordinate of the camera's origin

// Calculating distance to the center of a crack based on normalized coordinates (cx, cy) of the crack in the camera's field of view
float distanceToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - cameraXOrigin) * cameraFOVWidthMM;
  float y_mm = (cy - cameraYOrigin) * cameraFOVHeightMM;

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

  // Rotate back to forwards
  turnDegrees(-angle, turnSpeed);
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
  int targetPos = targetPosMM * gantryTicksPerMM;
  targetPos = constrain(targetPos, minGantryPosition, maxGantryPosition);
  long startLeft = readEncoderGantry();

  while (true) {
    long current = readEncoderGantry();
    long error = targetPos - current;

    bool atTarget = abs(error) <= encoderToleranceTicks;

    // Serial.print("Gantry Error: ");
    // Serial.print(error);
    if (atTarget) {
      stopGantry();
      return true;
    }
    
    pidControllerGantry.UpdateError(error);
    double output = pidControllerGantry.TotalError();
    // Serial.print(" Gantry PID Output: ");
    // Serial.print(output);

    int speed = atTarget ? 0 : pidOutputToSpeed(output, error, maxSpeed);
    setGantryPower(speed);
    // Serial.print(" Gantry Speed: ");
    // Serial.println(speed);
    delay(movementLoopDelayMs);
  }
}

bool isAtTargetPositionGantry() {
  return abs(targetPositionGantry - encoderValueGantry) < 5;
}

bool honeGantryPosition() {
  int previousGantryPos = readEncoderGantry();
  setGantryPower(-50); // Move gantry backward slowly
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

// MARK: Encoder Functions

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

bool setExtruderPosition(int targetPos, int maxSpeed){
  targetPos = constrain(targetPos, minGantryPosition, maxGantryPosition);
  long startLeft = readEncoderGantry();

  while (true) {
    long current = readEncoderGantry();
    long error = targetPos - current;

    bool atTarget = abs(error) <= encoderToleranceTicks;

    Serial.print("Gantry Error: ");
    Serial.print(error);
    if (atTarget) {
      stopGantry();
      return true;
    }
    
    pidControllerGantry.UpdateError(error);
    double output = pidControllerGantry.TotalError();
    Serial.print(" Gantry PID Output: ");
    Serial.print(output);

    int speed = atTarget ? 0 : pidOutputToSpeed(output, error, maxSpeed);
    setGantryPower(speed);
    Serial.print(" Gantry Speed: ");
    Serial.println(speed);
    delay(movementLoopDelayMs);
  }
}

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

void updateEncoderGantry(){
  if (digitalRead(ENCGANA) > digitalRead(ENCGANB))
    encoderValueGantry++;
  else
    encoderValueGantry--;
}

void updateEncoderExtruder(){
  if(digitalRead(ENCEXTA) > digitalRead(ENCEXTB)){
    encoderValueExtruder++;
  } else{
    encoderValueExtruder--;
  }
}

bool crackAuto() {
  while(true) {
    if(cx == -1 && cy == -1) {
      Serial.println("No Crack Detected");
      return false;
    }
    if(isDriveAligned()) {
      Serial.println("Drive aligned with crack center");
      if(isAtTargetPositionGantry()) {
        // TODO: Add extrusion
        return true;
      } else {
        while (!isAtTargetPositionGantry()) {
          gantryAlign(cx);
        }
        return true;
      }
    }
    
    else {
      int dCX = cx;
      int dCY = cy;
      while(!isDriveAligned){
        driveToCrackCenter(dCX, dCY);
      }
    }
    return false;
  }
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

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCGANA), updateEncoderGantry, RISING);
  
  pidController.Init(kP, kI, kD);
  pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);

  servo.attach(servoPin);
  servo.writeMicroseconds(1100); // send "stop" signal to ESC. Also necessary to arm the ESC.
  Serial.println("ESC TEST PREP");
  
  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);
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
int edfIncrementMax = 5;
int edfIncrement = 0;
int joystickDeadzone = 50;
bool killEverything = false;
bool joystickConnected = false;

void loop() {

  honeGantryPosition();
  driveToCrackCenter(0.5, 0.5);
  gantryAlign(0.5);
  // crackAuto();
  while (true){}
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
  // forwards();

  // MARK: Joystick Code
  /*
  int leftVal=map(joystickData.vy1, 0, 4095, -255, 255);
  int rightVal=map(joystickData.vy3, 0, 4095, -255, 255);
  int gantryVal=map(joystickData.vx2, 0, 4095, -255, 255);
  if (abs(leftVal) < joystickDeadzone) {
    leftVal = 0;
  }
  if (abs(rightVal) < joystickDeadzone) {
    rightVal = 0;
  }
  if (abs(gantryVal) < joystickDeadzone) {
    gantryVal = 0;
  }
  bool button3State=joystickData.button3;
  bool button2State=joystickData.button2;
  bool button1State=joystickData.button1;

  edfIncrement = 0;
  if (joystickData.vy3 < edfJoystickDefault) {
    edfIncrement=map(joystickData.vy2, 0, edfJoystickDefault-200, -edfIncrementMax, 0);
  } else {
    edfIncrement=map(joystickData.vy2, edfJoystickDefault+100, 4095, 0, edfIncrementMax);
  }

  gantryVal = -gantryVal;

  edfIncrement = -edfIncrement;
  edfIncrement /= edfIncrementMax; // Normalize to -1 to 1
  if(button1State == 0){
    killEverything = false;
    // hasZeroedGantry = true;
    autonomous = false;
  } else if(button3State == 0){
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
    // else if (button2State == 0 && button1State == 0) {
    //   autonomous = true;
    // }
    setSpeed(leftVal, rightVal);
    setGantryPower(gantryVal);
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
  */
  
  // // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

  // // driveDistance(700, 255);

  // // Serial.print(String(encoderValueLeft));
  // // Serial.print(", ");
  // // Serial.print(String(encoderValueRight));
  // // Serial.print(", ");
  // // Serial.println(String(encoderValueGantry));

  // // digitalWrite(Gantry1, HIGH);
  // // digitalWrite(Gantry2, LOW);
  // // analogWrite(ENI, 100);

  // if (BS.available() > 0) {
  //   String rawData = Serial.readStringUntil('\n');
  //   rawData.trim();
  //   if (rawData.length() > 0) {
  //     int cxIndex = rawData.indexOf("cx");
  //     int cyIndex = rawData.indexOf("cy");
  //     int xIndex = rawData.indexOf("x1");
  //     String xData = rawData.substring(cxIndex + 4, cyIndex - cxIndex - 3);
  //     String yData = rawData.substring(cyIndex + 4, xIndex - cyIndex - 3);
  //     cx = xData.toFloat();
  //     cy = yData.toFloat();
  //   Serial.println(rawData);
  //   }
  // }

  // if(autonomous){
  //   if(cx == 0 && cy == 0){
  //   if(isDriveAligned()){
  //     Serial.println("Drive aligned with crack center");
  //     }
  //   } else {
  //     // driveToCrackCenter(cx, cy);
  //   }
  // }

  // turnDegrees(180 , 200);
  // turnDegrees2(90);


  // setSpeed(255, 255);

  // Serial.print(String((ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));
}

 
