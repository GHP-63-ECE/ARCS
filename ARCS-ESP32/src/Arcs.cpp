#include <Arduino.h>
// Motor A Connections
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Motor B Connections
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// Motor C Connections
const int ENC = 9;
const int IN5 = 8;
const int IN6 = 7;

// Motor D Connections
const int END = 3;
const int IN7 = 5;
const int IN8 = 4;


// Encoder Connections
const int ENCA = 2; // Encoder A pin for Motor A
const int ENCB = 3; // Encoder B pin for Motor B

void setup() {
  // Set all control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Turn off motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
}

void loop() {
  // Move both motors forward at maximum speed (255)
  directionForward();
  setSpeed(255, 255);
  delay(2000);
  
  // Stop motors for 1 second
  stopMotors();
  delay(1000);
  
  // Move both motors backward at half speed (127)
  directionBackward();
  setSpeed(127, 127);
  delay(2000);
  
  // Stop motors
  stopMotors();
  delay(1000);
}

// Function to run motors forward
void directionForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to run motors backward
void directionBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Function to set individual motor speeds (PWM values: 0 to 255)
void setSpeed(int speedA, int speedB) {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}


/*// Code copied from https://create.arduino.cc/projecthub/curiores/how-to-control-a-dc-motor-with-an-encoder-d1734c
// for discussion at https://forum.arduino.cc/t/programing-motor-with-encoder/962793/10
//
// DaveX added Serial.print(pwr*dir)
// In Wokwi, there's no functional motor or encoder,
// so try the Serial Plotter icon to see what it is trying to do

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 12 // YELLOW
#define ENCB 13 // WHITE
#define PWM 3
#define IN2 4
#define IN1 5

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  Serial.println("target pos");
}

void loop() {

  // set target position
  //int target = 1200;
  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwr*dir);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}*/