
#include <Arduino.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <BluetoothSerial.h>
#include <PID.h>
#include <esp_now.h>
#include <WiFi.h>

// # include <HardwareSerial.h>
PID pidController = PID();
PID leftDrivePID = PID();
PID rightDrivePID = PID();
PID pidControllerGantry = PID();

BluetoothSerial BS;

Adafruit_MPU6050 imu;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float accelX;
float accelY;
float accelZ;

double gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
double accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
double raw_ax, raw_ay, raw_az;
double raw_gx, raw_gy, raw_gz;
double gyroX_offset_rad, gyroY_offset_rad, gyroZ_offset_rad;

const int numSamples = 2000; // Number of readings to average




// Drivetrain PID values
double kP = 0.45;
double kI = 0.0035; 
double kD = 0.0;

double kP_gantry = 0.1;
double kI_gantry = 0;
double kD_gantry = 0;
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
void setGantryPower(int power);
void updateEncoderGantry();
void setGantryPosition(int targetPos);

//Raspberry Pi communication Pin Definitions
//#define RXD2 16  // GPIO16 as RX
//#define TXD2 17  // GPIO17 as TX

//HardwareSerial mySerial(2); // Use Serial2
 
// Motor Pins
// Left
const int PWML = 23;
const int L1 = 16;
const int L2 = 17;  
// const int L1 = 22;
// const int L2 = 21;



// Right
const int PWMR = 19;
const int R1 = 18;
const int R2 = 5;

int targetPositionRight;
int targetPositionLeft;
int targetPositionGantry;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

// Encoder Connections
const int ENCAFL = 35; // Encoder A pin for Front Left Motor
const int ENCBFL = 26; // Encoder B pin for Front Left Motor

const int ENCAFR = 33; // Encoder A pin for Front Right Motor
const int ENCBFR = 25; // Encoder B pin for Front Right Motor

/**@brief 1st Encoder Pin for Extruder */
const int ENCEXTA = 36;
/**@brief 2nd Encoder Pin for Extruder */
const int ENCEXTB = 39;

//Gantry Motor Pins
const int ENI = 25;
const int PWM_PIN1 = 27;
const int PWM_PIN2 = 26; 

//Extruder Motor Pins
const int PWM_EXT = 14;
const int EXT1 = 12;
const int EXT2 = 13;

int cx = 0;
int cy = 0;


volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;
volatile long encoderValueGantry = 0;

int powerValue = 1100;

const float wheelDiameter = 45.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float turnSlipCompensation = 1.0; // This is a fudge factor to account for the fact that the robot doesn't turn perfectly in place
const float trackWidth = 150.0; // mm - TODO
int movementSpeed = 128; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 178.0; // Speed for turning left/right (0-255) - TODO
const float DISTANCE_PER_TICK = (PI * wheelDiameter) / ticksPerRotation; // mm per encoder tick
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

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("VY1 ");
  Serial.println(joystickData.vy1);
  Serial.print("VY2 ");
  Serial.println(joystickData.vy2);
  Serial.print("VY3 ");
  Serial.println(joystickData.vy3);
  Serial.print("Button1: ");
  Serial.println(joystickData.button1);
  Serial.print("Button2: ");
  Serial.println(joystickData.button2);
  Serial.print("Button3: ");
  Serial.println(joystickData.button3);
  Serial.println();
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

// MARK: Drive & Crack Functions

void driveDistance(float distance, int speed) {
  long targetTicks = distanceToEncoderTicks(distance);
  runToEncoderTargets(targetTicks, targetTicks, speed);
}

void turnDegrees(float degrees, int speed) {
  float wheelTravelMm = trackWidth * turnSlipCompensation * PI * (degrees / 360.0);
  long targetTicks = distanceToEncoderTicks(wheelTravelMm);
  runToEncoderTargets(targetTicks, -targetTicks, speed);
}

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

// MARK: Gantry Functions

void gantryDirectionForward() {
  digitalWrite(PWM_PIN1, HIGH);
  digitalWrite(PWM_PIN2, LOW);
}

void gantryDirectionBackward() {
  digitalWrite(PWM_PIN1, LOW);
  digitalWrite(PWM_PIN2, HIGH);
}

void stopGantry() {
  digitalWrite(PWM_PIN1, LOW);
  digitalWrite(PWM_PIN2, LOW);
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

// MARK: Setup
void setup() {

  Serial.begin(115200);
  BS.begin("ARCS");

  // Set all control pins to outputs

  //  Left
  pinMode(PWML, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  //  Right
  pinMode(PWMR, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(ENI, OUTPUT);

  pinMode(PWM_EXT, OUTPUT);
  pinMode(EXT1, OUTPUT);
  pinMode(EXT2, OUTPUT);

  pinMode(ENCAFL, INPUT);
  pinMode(ENCBFL, INPUT);
  pinMode(ENCAFR, INPUT);
  pinMode(ENCBFR, INPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCEXTA), updateEncoderGantry, RISING);
  
  pidController.Init(kP, kI, kD);
  pidControllerGantry.Init(kP_gantry, kI_gantry, kD_gantry);
  // Turn off motors initially

  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  Serial.println("ESC TEST PREP");
  
  leftDrivePID.Init(kP, kI, kD);
  rightDrivePID.Init(kP, kI, kD);
  // Turn off motors initially
  stopAllMotors();

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
int pwmValue = 255;
const int edfPowerMin = 1100;
const int edfPowerMax = 1900;
int edfPower = edfPowerMin;
int edfIncrementMax = 5;

void loop() {

  int leftVal=map(joystickData.vy1, 0, 4095, -255, 255);
  int rightVal=map(joystickData.vy2, 0, 4095, -255, 255);
  int edfIncrement=map(joystickData.vy3, 0, 4095, -edfIncrementMax, edfIncrementMax);
  bool button1State=joystickData.button1;
  bool button2State=joystickData.button2;
  bool escButtonState=joystickData.button3;

  if (escButtonState == 0) {
    edfPower += edfIncrement;
    edfPower = constrain(edfPower, edfPowerMin, edfPowerMax);
    servo.writeMicroseconds(edfPower); // output to edfs
  }
  
  setSpeed(leftVal, rightVal);

  // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

  // Serial.print(String(encoderValueLeft));
  // Serial.print(", ");
  // Serial.println(String(encoderValueRight));

  driveDistance(700, 255);

  Serial.print(String(encoderValueLeft));
  Serial.print(", ");
  Serial.print(String(encoderValueRight));
  Serial.print(", ");
  Serial.println(String(encoderValueGantry));

  digitalWrite(PWM_PIN1, HIGH);
  digitalWrite(PWM_PIN2, LOW);
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
    Serial.println(rawData);
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

 
