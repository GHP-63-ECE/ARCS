#include <Arduino.h>
#include <drive.h>
#include <HardwareSerial.h>

//Raspberry Pi communication Pin Definitions
#define RXD2 16  // GPIO16 as RX
#define TXD2 17  // GPIO17 as TX

HardwareSerial mySerial(2); // Use Serial2

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
const int ENCAFL = 2; // Encoder A pin for Motor A
const int ENCBFL = 3; // Encoder B pin for Motor A

const int ENCAFR = 3; // Encoder A pin for Motor B
const int ENCBFR = 4; // Encoder B pin for Motor B

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float circumference = wheelDiameter * PI;




void driveDistance(float distance, int speed) {

}

/// MARK: Setup
void setup() {

  Serial.begin(115200);  // Initialize Serial Monitor for debugging
  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2); // Initialize Serial2 with defined pins
  mySerial.println("Wake up from the matrix!");

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


/// MARK: Main Loop
void loop() {

    if (Serial.available() > 0) {
    String dataFromPi = Serial.readStringUntil('\n');
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
        mySerial.println("Unknown command received: " + dataFromPi);
        break;
    }
    }
  }

}
