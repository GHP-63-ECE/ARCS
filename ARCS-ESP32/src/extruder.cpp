#include <Arduino.h>
// #include <PID.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <SPI.h>

BluetoothSerial BS;

//Gantry pins
const int ENCODER_PIN_A1 = 0;
const int ENCODER_PIN_B1 = 0;
const int PWM_PIN1 = 27;
u_int8_t ENI = 14;
const int PWM_PIN2 = 26; 
const int pwmFreq = 5000; 
const int pwmResolution = 8; 
const int pwmChannel = 0;

const int STANDARD_DISTANCE = 0;

// int depth;

//Spray pins
// const int PWM_PIN2 = 0;
// PID pidController = PID();

// const int kP = 0;
// const int kI = 0;
// const int kD = 0;

int motorPosition = 0; 

int targetPosition = 0;


int encoderAValue;
int encoderBValue;

// void setMotorPosition(int targetPos);

void spray();
void setPowerGantry(int power);

int pw = 100;


void setup() {
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(ENI, OUTPUT);

//   ledcSetup(pwmChannel, pwmFreq, pwmResolution);
//     ledcAttachPin(ENI, pwmChannel);

  Serial.begin(115200);
  Serial.println("Starting Bluetooth Serial");
  BS.begin("ARCS");

//   ir.begin();
//   depth = ir.readRange();

//   pidController.Init(kP, kI, kD); 
}

void loop() {
    // encoderAValue = digitalRead(ENCODER_PIN_A1);
    // encoderBValue = digitalRead(ENCODER_PIN_B1);

    // if (encoderAValue == HIGH && encoderBValue == LOW) {
    //     motorPosition++;
    // } else if (encoderAValue == LOW && encoderBValue == HIGH) {
    //     motorPosition--;
    // }

    if(BS.available()){
        char inc = BS.read();
        if(inc == 'L'){
            BS.println("Left");
            setPowerGantry(-pw);
        } else if (inc == 'R'){
            BS.println("Right");
            setPowerGantry(pw);
        } else if (inc == 'S'){
            setPowerGantry(0);
        } else if (inc == '+'){
            pw += 10;
            pw = min(pw, 255);
            BS.println(pw);
        } else if (inc == '-'){
            pw -= 10;
            pw = max(pw, 0);
            BS.println(pw);
        }
    }
    // depth = ir.readRange();
}

// void setMotorPosition(int targetPos) {
//     int error = targetPos - motorPosition;
//     targetPosition = targetPos;

//     pidController.UpdateError(error);

//     analogWrite(PWM_PIN1, pidController.p_error*kP + pidController.i_error*kI + pidController.d_error*kD);    
// }

void spray(){
    // ledcWrite(pwmChannel, 100);
    digitalWrite(PWM_PIN1, LOW);
    digitalWrite(PWM_PIN2, HIGH);
    Serial.println("Spraying");

}

void setPowerGantry(int power) { 

    // ledcWrite(pwmChannel, abs(power));
    if(power == 0) {
        digitalWrite(PWM_PIN1, LOW);
        digitalWrite(PWM_PIN2, LOW);
        // analogWrite(ENI, 0);
    } else if (power < 0) {
        power = -power;
        digitalWrite(PWM_PIN1, HIGH);
        digitalWrite(PWM_PIN2, LOW);
    } else {
        digitalWrite(PWM_PIN1, LOW);
        digitalWrite(PWM_PIN2, HIGH);
    }

    analogWrite(14, 100);



}