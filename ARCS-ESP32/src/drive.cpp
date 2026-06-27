#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>


BluetoothSerial BS;
//#include <HardwareSerial.h>

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

//Raspberry Pi communication Pin Definitions
//#define RXD2 16  // GPIO16 as RX
//#define TXD2 17  // GPIO17 as TX

//HardwareSerial mySerial(2); // Use Serial2
 
// Motor Pins
// Left
const int PWML = 23;
const int L1 = 22;
const int L2 = 21;

// Right
const int PWMR = 19;
const int R1 = 18;
const int R2 = 5;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

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

int powerValue = 1100;


void setup() {

  Serial.begin(115200);
  BS.begin("ARCS");
  BS.print("Started:");
  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  BS.println("ESC TEST PREP");
  delay(7000);
   // Initialize Serial Monitor for debugging
  //mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2); // Initialize Serial2 with defined pins
  //mySerial.println("Wake up from the matrix!");

  // Set all control pins to outputs

  //  Left
  pinMode(PWML, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  //  Right
  pinMode(PWMR, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);

  // Set encoder pins to interrupts
  attachInterrupt(digitalPinToInterrupt(ENCAFL), updateEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFL), updateEncoderLeft, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCAFR), updateEncoderRight, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBFR), updateEncoderRight, RISING);
  
  // Turn off motors initially
 stopAllMotors();
}

void loop() {

    if (BS.available() > 0) {
    char dataFromPi = BS.read();
       switch(dataFromPi) {
      case 'w':
        forwards();
        BS.print('w');
        break;
      case 's':
        backwards();
        BS.print('s');
        break;
      case 'a':
        left();
        BS.print('a');
        break;
      case 'd':
        right();
        BS.print('d');
        break;
      case ' ':
        stopAllMotors();
        BS.print("Stop");
        break;

      case '+':
        powerValue += 100;
        BS.println(powerValue);
        break;
      case '-':
        powerValue -= 100;
        BS.println(powerValue);
        break;
      case 'k':
        powerValue = 0;
        BS.println(powerValue);
        break;
      default:
        BS.println("Unknown command received: " + dataFromPi);
        break;
    }
    int pwmVal = map(powerValue,0, 1023, 1100, 1900); // translate POT values to ESC value.
    float percentVal = ((pwmVal - 1100) / 8);
    servo.writeMicroseconds(pwmVal);
    }
  }


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
