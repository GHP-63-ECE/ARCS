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
const int PWMFL = 9;
const int FL1 = 8;
const int FL2 = 7;

// Front Right
const int PWMFR = 3;
const int FR1 = 5;
const int FR2 = 4;

// Back Left
const int PWMBL = 9;
const int BL1 = 8;
const int BL2 = 7;

// Back Right
const int PWMBR = 3;
const int BR1 = 5;
const int BR2 = 4;

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
        directionForward();
        setSpeed(255, 255);
        break;
      case 'B':
        directionBackward();
        setSpeed(255, 255);
        break;
      case 'L':
        directionForward();
        setSpeed(255, 100); // Left turn: slower left motor
        break;
      case 'R':
        directionForward();
        setSpeed(100, 255); // Right turn: slower right motor
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

void driveDistance(float distance, int speed) {

}


void stopAllMotors(){
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, LOW);
  digitalWrite(BL1, LOW);
  digitalWrite(BL2, LOW);
  digitalWrite(BR1, LOW);
  digitalWrite(BR2, LOW);
}

void directionForward(){
  digitalWrite(FL1, HIGH);
  digitalWrite(FL2, LOW);
  digitalWrite(FR1, HIGH);
  digitalWrite(FR2, LOW);
}

void directionBackward(){
  digitalWrite(FL1, LOW);
  digitalWrite(FL2, HIGH);
  digitalWrite(FR1, LOW);
  digitalWrite(FR2, HIGH);
}

void setSpeed(int speedA, int speedB){
  analogWrite(PWMFL, speedA);
  analogWrite(PWMFR, speedB);
  analogWrite(PWMBL, speedA);
  analogWrite(PWMBR, speedB);
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
