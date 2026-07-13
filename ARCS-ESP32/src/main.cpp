#include <Arduino.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
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
const int Gantry1 = 26;
const int Gantry2 = 27; 

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

// MARK: Variables

float angle = 0;

bool autonomous = false;
bool prevButton4State = false;
bool prevButton5State = false;

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

long fixedExtrudeTicks = 1000;
int pwrExt = 100;
int pwrGantry = 100;

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
  // Serial.println(power);
  analogWrite(ENI, power);
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
  digitalWrite(EXT1, HIGH);
  digitalWrite(EXT2, HIGH);
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

// MARK: Setup
void setup() {

  Serial.begin(115200);
  // BS.begin("ARCS");

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
  
  servo.attach(servoPin);
  servo.writeMicroseconds(1100); // send "stop" signal to ESC. Also necessary to arm the ESC.
  Serial.println("ESC TEST PREP");

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
int joystickDeadzone = 100;
bool killEverything = false;
bool joystickConnected = true;

void loop() {
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
  if (joystickData.vy2 < edfJoystickDefault) {
    edfIncrement=map(joystickData.vy2, 0, edfJoystickDefault-200, -edfIncrementResolution, 0);
  } else {
    edfIncrement=map(joystickData.vy2, edfJoystickDefault+100, 4095, 0, edfIncrementResolution);
  }

  gantryVal = -gantryVal;
  leftVal = -leftVal;
  rightVal = -rightVal;

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
    } else if (button4State == 0){
      setExtruderPower(extVal);
    }

    // Serial.print("Setting Speed");
    setSpeed(leftVal, rightVal);
    setGantryPower(gantryVal);

  }

  if(button4State == 1){
    stopExtruder();
  }
    
  // Serial.println(String(encoderValueGantry));
  // Serial.println(encoderValueLeft);
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
  Serial.print(" Extruder Val: ");
  Serial.print(extVal);
  Serial.print(" Kill Everything: ");
  Serial.print(killEverything);
  Serial.println();
}