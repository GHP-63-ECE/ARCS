#include <Arduino.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <BluetoothSerial.h>
#include <PID.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_log.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>


// MARK: Function prototypes 
// because c++ is a liar
void stopAllMotors();
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
void setGantryPower(int power);
void updateEncoderGantry();
void setGantryPosition(int targetPos);

// MARK: Pinout

// Left Motor Pins
const int PWML = 23; 
const int L1 = 16; // const int L1 = 22;
const int L2 = 17; // const int L2 = 21;

// Right Motor Pins
const int PWMR = 19; 
const int R1 = 18;
const int R2 = 5;

// Encoder Connections
const int ENCAFL = 35; // Encoder A pin for Front Left Motor
const int ENCBFL = 26; // Encoder B pin for Front Left Motor

const int ENCAFR = 33; // Encoder A pin for Front Right Motor
const int ENCBFR = 25; // Encoder B pin for Front Right Motor

const int ENCEXTA = 36; // 1st Encoder Pin for Extruder
const int ENCEXTB = 39; // 2nd Encoder Pin for Extruder

// Gantry Motor Pins
const int ENI = 25;
const int Gantry1 = 27;
const int Gantry2 = 26; 

// Extruder Motor Pins
const int PWM_EXT = 14;
const int EXT1 = 12;
const int EXT2 = 15;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

//Acclerometer object declara
Adafruit_LSM6DSOX lsm6ds;

// MARK: Variables

PID pidController = PID();
PID leftDrivePID = PID();
PID rightDrivePID = PID();
PID pidControllerGantry = PID();

BluetoothSerial BS;

static const char* TAG = "ARCS-ESP32";

// Drivetrain PID values
double kP = 0.45;
double kI = 0.0035; 
double kD = 0.0;

double kP_gantry = 0.1;
double kI_gantry = 0;
double kD_gantry = 0;
int cx = 0;
int cy = 0;

bool autonomous = false;


volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;
volatile long encoderValueGantry = 0;
int targetPositionRight;
int targetPositionLeft;
int targetPositionGantry;

const float wheelDiameter = 45.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float turnSlipCompensation = 1.0; // This is a fudge factor to account for the fact that the robot doesn't turn perfectly in place
const float trackWidth = 150.0; // mm - TODO
int movementSpeed = 255; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 200; // Speed for turning left/right (0-255) - TODO
const int encoderToleranceTicks = 15;
const int minimumPIDSpeed = 55;
const unsigned long movementLoopDelayMs = 10;

const float cameraFOVWidthMM = 100.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 56.0; // Height of the camera's field of view in millimeters - TODO

int pwrGantry = 100;
int pwrExt = 100;

// MARK: ESP-NOW Comms

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
int vy1;
int vy2;
int vy3;
bool button1;
bool button2;
bool button3;
} struct_message;

// Create a struct_message called joystickData
struct_message joystickData;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  ESP_LOGD(TAG, "VY1: %d", joystickData.vy1);
  ESP_LOGD(TAG, "VY2: %d", joystickData.vy2);
  ESP_LOGD(TAG, "VY3: %d", joystickData.vy3);
  ESP_LOGD(TAG, "Button1: %d", joystickData.button1);
  ESP_LOGD(TAG, "Button2: %d", joystickData.button2);
  ESP_LOGD(TAG, "Button3: %d", joystickData.button3);
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

    ESP_LOGD(TAG, "Left Error: %ld", leftError);
    ESP_LOGD(TAG, "Right Error: %ld", rightError);
    ESP_LOGD(TAG, "Left Target: %ld", targetPositionLeft);
    ESP_LOGD(TAG, "Right Target: %ld", targetPositionRight);

    if (leftAtTarget && rightAtTarget) {
      stopAllMotors();
      return true;
    }

    if (millis() - startTime > timeoutMs) {
      stopAllMotors();
      ESP_LOGW(TAG, "Encoder movement timed out");
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

    ESP_LOGD(TAG, "Left Output: %f", leftOutput);
    ESP_LOGD(TAG, "Right Output: %f", rightOutput);

    int leftSpeed = leftAtTarget ? 0 : pidOutputToSpeed(leftOutput, leftError, maxSpeed);
    int rightSpeed = rightAtTarget ? 0 : pidOutputToSpeed(rightOutput, rightError, maxSpeed);
    if (leftTargetTicks == -rightTargetTicks) {
      // If turning, ensure both motors are moving at the same speed
      setSpeed(leftSpeed, -leftSpeed);
      ESP_LOGD(TAG, "Output Speed: %d", leftSpeed);
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
float cameraYOrigin = 0.5; // Normalized Y coordinate of the camera's origin

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
  const float distanceThreshold = 50; // mm
  const float angleThreshold = 5.0; // degrees

  // Check if the robot is aligned with the crack center
  return (abs(distance) < distanceThreshold) && (abs(angle) < angleThreshold);
 }

void gantryAlign(float cx){
  
  // Convert normalized pixel coordinates to millimeters
  float x_mm = ((cx - 0.5) * cameraFOVWidthMM)+100;

  // Calculate the distance to move the gantry based on the x_mm value
  float distanceToMove = x_mm; // Assuming 1:1 mapping for simplicity, adjust as necessary

  // Move the gantry to align with the crack center
  setGantryPosition(distanceToMove);
}

// Drives to the crack's center such that the crack is centered in the camera's field of view at (0.5, 0.5) 
void driveToCrackCenter(float cx, float cy) {
  float distance = distanceToCrackCenter(cx, cy);
  float angle = angleToCrackCenter(cx, cy);

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
  digitalWrite(Gantry1, LOW);
  digitalWrite(Gantry2, LOW);
  analogWrite(ENI, 0);
}

void setGantryPower(int power) {
  if (power == 0) {
    stopGantry();
  } else if (power < 0) {
    power = -power;
    gantryDirectionBackward();
  } else {
    gantryDirectionForward();
  }
  analogWrite(ENI, power);
}

void setGantryPosition(int targetPos) {
    int error = targetPos - encoderValueGantry;
    targetPositionGantry = targetPos;

    pidControllerGantry.UpdateError(error);

    analogWrite(ENI, pidControllerGantry.p_error*kP_gantry + pidControllerGantry.i_error*kI_gantry + pidControllerGantry.d_error*kD_gantry);    
}

bool isAtTargetPositionGantry() {
    return abs(targetPositionGantry - encoderValueGantry) < 5;
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
  if (digitalRead(ENCEXTA) > digitalRead(ENCEXTB))
    encoderValueGantry++;
  else
    encoderValueGantry--;
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
  pinMode(ENCEXTA, INPUT);
  pinMode(ENCEXTB, INPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCEXTA), updateEncoderGantry, RISING);
  
  pidController.Init(kP, kI, kD);
  pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);

  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  ESP_LOGI(TAG, "ESC TEST PREP");
  
  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);
  // Turn off motors initially
  stopAllMotors();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    ESP_LOGE(TAG, "Error initializing ESP-NOW");
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
int pwmValue = 255;

const int edfPowerMin = 500;
const int edfPowerMax = 2400;
const int edfJoystickDefault = 1950;
int edfPower = edfPowerMin;
int edfIncrementMax = 5;
int edfIncrement = 0;

void loop() {

  int leftVal=map(joystickData.vy1, 0, 4095, -255, 255);
  int rightVal=map(joystickData.vy2, 0, 4095, -255, 255);
  bool button3State=joystickData.button3;
  bool button2State=joystickData.button2;
  bool escButtonState=joystickData.button1;

  edfIncrement = 0;
  if (joystickData.vy3 < edfJoystickDefault) {
    edfIncrement=map(joystickData.vy3, 0, edfJoystickDefault-200, -edfIncrementMax, 0);
  } else {
    edfIncrement=map(joystickData.vy3, edfJoystickDefault+100, 4095, 0, edfIncrementMax);
  }


  if (escButtonState == 0) {
    edfPower += edfIncrement;
    edfPower = constrain(edfPower, edfPowerMin, edfPowerMax);
    
    servo.writeMicroseconds(edfPower); // output to edfs
  }

  if(button3State == 0){
    gantryAlign(cx);
  } 

  if(button2State == 0){
    setExtruderPower(pwrExt);
  }
  ESP_LOGD(TAG, "EDF Increment: %d", edfIncrement);
  ESP_LOGD(TAG, "EDF Power: %d", edfPower);
  
  setSpeed(leftVal, rightVal);

  // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

  // driveDistance(700, 255);

  ESP_LOGD(TAG, "Encoder Values - Left: %ld, Right: %ld, Gantry: %ld", encoderValueLeft, encoderValueRight, encoderValueGantry);

  digitalWrite(Gantry1, HIGH);
  digitalWrite(Gantry2, LOW);
  analogWrite(ENI, 100);

  if (BS.available() > 0) {
    String rawData = Serial.readStringUntil('\n');
    rawData.trim();
    if (rawData.length() > 0) {
      int cxIndex = rawData.indexOf("cx");
      int cyIndex = rawData.indexOf("cy");
      int xIndex = rawData.indexOf("x1");
      String xData = rawData.substring(cxIndex + 4, cyIndex - cxIndex - 3);
      String yData = rawData.substring(cyIndex + 4, xIndex - cyIndex - 3);
      cx = xData.toFloat();
      cy = yData.toFloat();
      ESP_LOGD(TAG, "Received data - cx: %f, cy: %f", cx, cy);
    }
  }

  if(autonomous){
    if(isDriveAligned()){
      gantryAlign(cx);
      if(isAtTargetPositionGantry()){
        setExtruderPower(pwrExt);
        delay(500);//time to extrude
        setExtruderPower(0);
        delay(500);//time to spray
        setExtruderPower(-pwrExt);
        delay(500);//time to retract
        ESP_LOGD(TAG, "Drive Aligned");
      } else {
        setExtruderPower(0);
      }
    } else {
      driveToCrackCenter(cx, cy);
    }
  }

  // turnDegrees(180 , 200);
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
  // while (true) {}

  // Serial.print(String((ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));
}

 
