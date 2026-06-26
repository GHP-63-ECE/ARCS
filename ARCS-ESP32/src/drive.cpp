#include <Arduino.h>
#include <BluetoothSerial.h>
#include "drive.h"

BluetoothSerial SerialBT;

//#include <HardwareSerial.h>

// Function prototypes because c++ is a liar
void stopAllMotors();
void directionForward();
void directionBackward();
void setSpeed(int speedA, int speedB);
void updateEncoderLeft();
void updateEncoderRight();
void driveDistance(float distance, int speed);

//Raspberry Pi communication Pin Definitions
//#define RXD2 16  // GPIO16 as RX
//#define TXD2 17  // GPIO17 as TX

//HardwareSerial mySerial(2); // Use Serial2
 
// Motor Pins
// Front Left
const int PWMFL = 23;
const int FL1 = 22;
const int FL2 = 1;

// Front Right
const int PWMFR = 18;
const int FR1 = 5;
const int FR2 = 17;

// Back Left
const int PWMBL = 3;
const int BL1 = 21;
const int BL2 = 19;

// Back Right
const int PWMBR = 16;
const int BR1 = 4;
const int BR2 = 2;

// Encoder Connections
const int ENCAFL = 32; // Encoder A pin for Front Left Motor
const int ENCBFL = 33; // Encoder B pin for Front Left Motor

const int ENCAFR = 12; // Encoder A pin for Front Right Motor
const int ENCBFR = 13; // Encoder B pin for Front Right Motor

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float circumference = wheelDiameter * PI;

const int movementSpeed = 128; // Speed for driving forward/backward (0-255)


void setup() {

  Serial.begin(115200);
  SerialBT.begin("ARCS"); // Name your Bluetooth device
  Serial.println("Bluetooth device is ready to pair!");
   // Initialize Serial Monitor for debugging
  //mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2); // Initialize Serial2 with defined pins
  //mySerial.println("Wake up from the matrix!");

  // Set all control pins to outputs

  // Front Left
  pinMode(PWMFL, OUTPUT);
  pinMode(FL1, OUTPUT);
  pinMode(FL2, OUTPUT);
  
  // Front Right
  pinMode(PWMFR, OUTPUT);
  pinMode(FR1, OUTPUT);
  pinMode(FR2, OUTPUT);
  
  // Back Left
  pinMode(PWMBL, OUTPUT);
  pinMode(BL1, OUTPUT);
  pinMode(BL2, OUTPUT);
  
  // Back Right
  pinMode(PWMBR, OUTPUT);
  pinMode(BR1, OUTPUT);
  pinMode(BR2, OUTPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFR), updateEncoderRight, RISING);
  
  // Turn off motors initially
 stopAllMotors();
}

void loop() {

    if (SerialBT.available() > 0) {
    String dataFromPi = SerialBT.readStringUntil('\n');
    if (dataFromPi.length() > 0) {
       switch(dataFromPi.charAt(0)) {
      case 'F':
        setSpeed(movementSpeed, movementSpeed);
        break;
      case 'B':
        setSpeed(-movementSpeed, -movementSpeed);
        break;
      case 'L':
        setSpeed(movementSpeed, -movementSpeed); 
        break;
      case 'R':
        setSpeed(-movementSpeed, movementSpeed); 
        break;
      case 'S':
        stopAllMotors();
        break;
      default:
        SerialBT.println("Unknown command received: " + dataFromPi);
        break;
    }
    }
  }

}

void stopLeftMotors() {
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, LOW);
  
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
}

void stopRightMotors() {
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);

  digitalWrite(BR1, LOW);
  digitalWrite(BR2, LOW);
}

void stopAllMotors() {
  stopLeftMotors();
  stopRightMotors();
}

void LeftMotorsForwards() {
  digitalWrite(FL1, HIGH);
  digitalWrite(FL2, LOW);

  digitalWrite(BL1, HIGH);
  digitalWrite(BL2, LOW);
}

void RightMotorsForwards() {
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);

  digitalWrite(BR1, HIGH);
  digitalWrite(BR2, LOW);
}

void LeftMotorsBackwards() {
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);

  digitalWrite(BL1, LOW);
  digitalWrite(BL2, HIGH);
}

void RightMotorsBackwards() {
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, HIGH);

  digitalWrite(BR1, LOW);
  digitalWrite(BR2, HIGH);
}

void setSpeed(int left, int right){
  if (left == 0) {
    stopRightMotors();
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
  analogWrite(PWMFL, left);
  analogWrite(PWMBL, left);

  analogWrite(PWMFR, right);
  analogWrite(PWMBR, right);
}


void updateEncoderLeft(){
    if (digitalRead(ENCAFL)> digitalRead(ENCBFL))
    encoderValueLeft++;
  else
    encoderValueLeft--;
}

void updateEncoderRight(){
   if (digitalRead(ENCAFR)> digitalRead(ENCBFR))
    encoderValueRight++;
  else
    encoderValueRight--;
}
