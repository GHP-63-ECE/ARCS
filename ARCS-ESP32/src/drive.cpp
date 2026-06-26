#include <Arduino.h>

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
const int FL2 = 21;

// Front Right
const int PWMFR = 19;
const int FR1 = 18;
const int FR2 = 5;

// Back Left
const int PWMBL = 4;
const int BL1 = 2;
const int BL2 = 15;

// Back Right
const int PWMBR = 14;
const int BR1 = 12;
const int BR2 = 13;

// Encoder Connections
const int ENCAFL = 34; // Encoder A pin for Front Left Motor
const int ENCBFL = 35; // Encoder B pin for Front Left Motor

const int ENCAFR = 36; // Encoder A pin for Front Right Motor
const int ENCBFR = 39; // Encoder B pin for Front Right Motor

volatile long encoderValueLeft = 0;
volatile long encoderValueRight = 0;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float circumference = wheelDiameter * PI;

const int movementSpeed = 128; // Speed for driving forward/backward (0-255)


void setup() {

  Serial.begin(115200);
  Serial.print("Started:");
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

    if (Serial.available() > 0) {
    char dataFromPi = Serial.read();
       switch(dataFromPi) {
      case 'w':
        setSpeed(movementSpeed, movementSpeed);
        Serial.print('w');
        break;
      case 's':
        setSpeed(-movementSpeed, -movementSpeed);
        Serial.print('s');
        break;
      case 'a':
        setSpeed(movementSpeed, -movementSpeed); 
        Serial.print('a');
        break;
      case 'd':
        setSpeed(-movementSpeed, movementSpeed); 
        Serial.print('d');
        break;
      case ' ':
        stopAllMotors();
        Serial.print("Stop");
        break;
      default:
        Serial.println("Unknown command received: " + dataFromPi);
        break;
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
