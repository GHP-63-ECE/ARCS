#include <ESP32Servo.h> // ONLY LIBRARY NECESARY FOR ESC 
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial BS;

byte servoPin = 13; // signal pin for the ESC.
Servo servo;

int powerVal = 1100;


void setup() {
  Serial.begin(115200);
  BS.begin("ARCSEDF");

  // ESC ARMING PROCESS //
  servo.attach(servoPin);
  servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
  Serial.println("ESC TEST PREP");
  delay(7000); // delay to allow the ESC to recognize the stopped signal.

}


void loop() {
    if(BS.available()){
        char inc = BS.read();
        if(inc == '+'){
            powerVal += 100;
            BS.println(powerVal);
        } else if (inc == '-'){
            powerVal -= 100;
            BS.println(powerVal);
        } else if (inc == 'k'){
            powerVal = 0;
            BS.println(powerVal);
        }
    }
  int pwmVal = map(powerVal,0, 1023, 1100, 1900); // translate POT values to ESC value.
  float percentVal = ((pwmVal - 1100) / 8);
  servo.writeMicroseconds(pwmVal); // Send signal to ESC.
  delay(50);
}