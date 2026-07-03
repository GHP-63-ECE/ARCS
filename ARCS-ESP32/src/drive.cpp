#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>

#include <PID.h>

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

//#include <HardwareSerial.h>
PID pidController = PID();
PID pidControllerGantry = PID();


// Drivetrain PID values
double kP = 0;
double kI = 0;
double kD = 0;

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
void forwards();
void backwards();
void left();
void right();
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
const int ENCBFL = 34; // Encoder B pin for Front Left Motor

const int ENCAFR = 33; // Encoder A pin for Front Right Motor
const int ENCBFR = 32; // Encoder B pin for Front Right Motor

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

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float trackWidth = 150.0; // mm - TODO
int movementSpeed = 128; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 128.0; // Speed for turning left/right (0-255) - TODO

const float cameraFOVWidthMM = 100.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 56.0; // Height of the camera's field of view in millimeters - TODO

int pwrGantry = 100;
int pwrExt = 100;


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
  float x_mm = ((cx - 0.5) * cameraFOVWidthMM)+100;
  float y_mm = ((cy - 0.5) * cameraFOVHeightMM)+180;

  // Calculate distance to the center of the crack using Pythagorean theorem
  return sqrt(x_mm * x_mm + y_mm * y_mm);
}

float angleToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = ((cx - 0.5) * cameraFOVWidthMM)+100;
  float y_mm = ((cy - 0.5) * cameraFOVHeightMM)+180;

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

  // Rotate to face the crack center
  rotateDegreesWithoutEncoders(angle, turnSpeed);

  // Drive forward to the crack center
  driveDistanceWithoutEncoders(distance, movementSpeed);
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

      if(!imu.begin()){
        Serial.print("IMU not found");
    }

    imu.setAccelerometerRange(MPU6050_RANGE_2_G);
    imu.setGyroRange(MPU6050_RANGE_2000_DEG);

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


  stopAllMotors();
}

// MARK: Loop

void loop() {
  // Serial.println(encoderValueLeft + " hello " + encoderValueRight);

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

  // Serial.print(String(digitalRead(ENCAFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCBFR)));
  // Serial.print(", ");
  // Serial.print(String(digitalRead(ENCAFL)));
  // Serial.print(", ");
  // Serial.println(String(digitalRead(ENCBFL)));


    // if (BS.available() > 0) {
    // char dataFromPi = BS.read();
    //    switch(dataFromPi) {
    //   case 'w':
    //     forwards();
    //     BS.print('w');
    //     break;
    //   case 's':
    //     backwards();
    //     BS.print('s');
    //     break;
    //   case 'a':
    //     left();
    //     BS.print('a');
    //     break;
    //   case 'd':
    //     right();
    //     BS.print('d');
    //     break;
    //   case ' ':
    //     stopAllMotors();
    //     BS.print("FUCK! Stop");
    //     break;

    //   case '+':
    //     powerValue += 100;
    //     BS.println(powerValue);
    //     break;
    //   case '-':
    //     powerValue -= 100;
    //     BS.println(powerValue);
    //     break;
    //   case 'k':
    //     powerValue = 0;
    //     BS.println(powerValue);
    //     break;
    //   case 'l':
    //     setGantryPower(pwrGantry);
    //     BS.println("Gantry Forward");
    //     break;
    //   case 'r':
    //     setGantryPower(-pwrGantry);
    //     BS.println("Gantry Backward");
    //     break;
    //   case 'e':
    //     setGantryPower(0);
    //     BS.println("Gantry Stop");
    //     break;
    //   case 'p':
    //     pwrGantry += 10;
    //     if (pwrGantry > 255) pwrGantry = 255;
    //     BS.println("Power: " + String(pwrGantry));
    //     break;
    //   case 'm':
    //     pwrGantry -= 10;
    //     if (pwrGantry < 0) pwrGantry = 0;
    //     BS.println("Power: " + String(pwrGantry));
    //     break;
    //   case 'v':
    //    movementSpeed += 10;
    //    if (movementSpeed > 255) movementSpeed = 255;
    //    BS.println("Drive Speed: " + String(movementSpeed));
    //   break;
    //   case 'b':
    //     movementSpeed -= 10;
    //       if (movementSpeed < 0) movementSpeed = 0;
    //     BS.println("Drive Speed: " + String(movementSpeed));
    //   break;
    //   case 'y':
    //   driveToCrackCenter(0.6, 0.5); // Example coordinates for the crack center
    //   BS.println("Driving to Crack Center, godspeed");
    //   break;
    //   case 'g':
    //     gantryAlign(0.6); // Example x-coordinate for the crack center
    //     BS.println("Aligning Gantry to Crack Center");
    //     break;
    //   default:
    //     BS.println("Unknown command received: " + dataFromPi);
    //     break;
    // }
    // int pwmVal = map(powerValue,0, 1023, 1100, 1900); // translate POT values to ESC value.
    // float percentVal = ((pwmVal - 1100) / 8);
    // servo.writeMicroseconds(pwmVal);
    // delay(50);
    // }

    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    accelX = accel.acceleration.x - 0.4;
    accelY = accel.acceleration.y + 0.1;
    accelZ = accel.acceleration.z - 0.84;

    Serial.print("Accel X:");
    Serial.print(accelX);
    Serial.print(", Y:");
    Serial.print(accelY);
    Serial.print(", Z:");
    Serial.print(accelZ);
    Serial.print("m/s^2 ");

    gyroX += gyro.gyro.x*0.01;
    gyroY += gyro.gyro.y*0.01;
    gyroZ += gyro.gyro.z*0.01;

    Serial.print("Gyro X:");
    Serial.print(gyroX);
    Serial.print(", Y:");
    Serial.print(gyroY);
    Serial.print(", Z:");
    Serial.print(gyroZ);
    Serial.println("rad");

    delay(10);
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