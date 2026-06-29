#include <Arduino.h>
#include <BluetoothSerial.h>
#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>

#define RX2_PIN 16
#define TX2_PIN 17
#define SERIAL_BAUD 115200

String inputString = "";
bool stringComplete = false;

float cx;     // Centroid X
float cy;     // Centroid Y
float conf;

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

int powerValue = 1100;

const float wheelDiameter = 44.0; // mm
const long ticksPerRotation = 7*298; 
const float wheelCircumference = wheelDiameter * PI;
const float trackWidth = 234.0; // mm
float movementSpeed = 128.0; // Speed for driving forward/backward (0-255)
const float rpmAtMaxSpeed = 100; // Maximum RPM of the motor at full speed - TODO
const float turnSpeed = 128.0; // Speed for turning left/right (0-255)

const float cameraFOVWidthMM = 100.0; // Width of the camera's field of view in millimeters - TODO
const float cameraFOVHeightMM = 75.0; // Height of the camera's field of view in millimeters - TODO



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
  float x_mm = (cx - 0.5) * cameraFOVWidthMM;
  float y_mm = (cy - 0.5) * cameraFOVHeightMM;

  // Calculate distance to the center of the crack using Pythagorean theorem
  return sqrt(x_mm * x_mm + y_mm * y_mm);
}

float angleToCrackCenter(float cx, float cy) {
  // Convert normalized pixel coordinates to millimeters
  float x_mm = (cx - 0.5) * cameraFOVWidthMM;
  float y_mm = (cy - 0.5) * cameraFOVHeightMM;

  // Calculate angle to the center of the crack using arctangent
  return atan2(x_mm, -y_mm) * (180 / PI); // atan2 is flipped so that 0 degrees is forward and positive angles are to the right
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

void parsePiData(String jsonString);


void setup() {

  Serial.begin(115200);
  BS.begin("ARCS");
  BS.print("Started:");
  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  BS.println("ESC TEST PREP");
  delay(7000);

  Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  inputString.reserve(2048);
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

  // Read incoming data packets line-by-line
  if (Serial.available() > 0) {
    String incomingString = Serial.readStringUntil('\n');
    incomingString.trim(); // Clean up any hidden whitespace

    // Find the comma separator between cx and cy
    int commaIndex = incomingString.indexOf(',');

    if (commaIndex != -1) {
      // Split the data string
      String cxStr = incomingString.substring(0, commaIndex);
      String cyStr = incomingString.substring(commaIndex + 1);

      // Convert normalized text data back into floating-point numbers
      float cx = cxStr.toFloat();
      float cy = cyStr.toFloat();

      driveToCrackCenter(cx, cy); // Call the function to drive to the crack center
    }
  }

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
      case 'p':
        driveToCrackCenter(cx, cy);
        BS.println("pathing");
        break;
      default:
        BS.println("Unknown command received: " + dataFromPi);
        break;
    }
    int pwmVal = map(powerValue,0, 1023, 1100, 1900); // translate POT values to ESC value.
    float percentVal = ((pwmVal - 1100) / 8);
    servo.writeMicroseconds(pwmVal);
    }

    while (Serial2.available()) {
    char inChar = (char)Serial2.read();
    
    // The Pi script ends every JSON packet with a newline character (\n)
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  // 2. Parse the JSON packet when a complete line is received
  if (stringComplete) {
    parsePiData(inputString);
    
    // Reset the buffer for the next incoming packet
    inputString = "";
    stringComplete = false;
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

void parsePiData(String jsonString) {
  // DynamicJsonDocument allocates memory on the stack (v6/v7 compatible layout)
  // 2048 bytes is plenty of room for several detected cracks in one frame
  StaticJsonDocument<2048> doc;

  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    Serial.print("JSON Deserialization failed: ");
    Serial.println(error.f_str());
    return;
  }

  // Extract core envelope variables
  long frameId = doc["f"];       // Frame ID
  int numDetections = doc["n"];  // Number of detections in this frame

  // Print frame info to ESP32 console
  Serial.print("Frame: ");
  Serial.print(frameId);
  Serial.print(" | Found: ");
  Serial.print(numDetections);
  Serial.println(" cracks");

  // If there are detections, loop through and pull relative coordinates
  if (numDetections > 0) {
    JsonArray detections = doc["d"];
    
    for (int i = 0; i < detections.size(); i++) {
      JsonObject d = detections[i];
      
      // Pull coordinates (0.0000 to 1.0000 relative to camera width/height)
      cx = d["cx"];     // Centroid X
      cy = d["cy"];     // Centroid Y
      conf = d["conf"]; // Confidence score (0.0 to 1.0)

      // Print parsed coordinates out
      Serial.print("  -> Crack #");
      Serial.print(i);
      Serial.print(" | Center: (");
      Serial.print(cx, 4);
      Serial.print(", ");
      Serial.print(cy, 4);
      Serial.print(") | Confidence: ");
      Serial.println(conf, 3);

      // --- YOUR ESP32 CONTROL LOGIC GOES HERE ---
      // For example:
      // if (cx < 0.45) { Turn Left; }
      // else if (cx > 0.55) { Turn Right; }
      // else { Stay Centered; }
    }
  }
}
