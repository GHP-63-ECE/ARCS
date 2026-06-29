#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

Adafruit_MPU6050 imu;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float accelX;
float accelY;
float accelZ;

double gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
double accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
double raw_ax, raw_ay, raw_az;
double raw_gx, raw_gy, raw_gz;
double gyroX_offset_rad, gyroY_offset_rad, gyroZ_offset_rad;

const int numSamples = 2000; // Number of readings to average



void readRawIMU() {
  Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
  Wire.write(0x3B); // Starting register for Accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_I2CADDR_DEFAULT, 14, true); // Request 14 registers

  raw_ax = Wire.read() << 8 | Wire.read();
  raw_ay = Wire.read() << 8 | Wire.read();
  raw_az = Wire.read() << 8 | Wire.read();
  
  int16_t temp = Wire.read() << 8 | Wire.read(); // Skip temperature data
  
  raw_gx = Wire.read() << 8 | Wire.read();
  raw_gy = Wire.read() << 8 | Wire.read();
  raw_gz = Wire.read() << 8 | Wire.read();
}


void setup(){
    Serial.begin(115200);
    if(!imu.begin()){
        Serial.print("IMU not found");
    }

    imu.setAccelerometerRange(MPU6050_RANGE_2_G);
    imu.setGyroRange(MPU6050_RANGE_2000_DEG);

    // Run the calibration loop
  for (int i = 0; i < numSamples; i++) {
    readRawIMU();
    
    // Accumulate the errors
    gyroX_offset += raw_gx;
    gyroY_offset += raw_gy;
    gyroZ_offset += raw_gz;
    
    accelX_offset += raw_ax;
    accelY_offset += raw_ay;
    accelZ_offset += raw_az;
    
    delay(2); // Short gap between readings
  }
gyroX_offset /= numSamples;
  gyroY_offset /= numSamples;
  gyroZ_offset /= numSamples;

  accelX_offset /= numSamples;
  accelY_offset /= numSamples;
  accelZ_offset /= numSamples;

  Serial.print("Gyro Offsets - X: ");
  Serial.print(gyroX_offset);
  Serial.print(", Y: ");
  Serial.print(gyroY_offset);
  Serial.print(", Z: ");
  Serial.println(gyroZ_offset);

  Serial.print("Accel Offsets - X: ");
  Serial.print(accelX_offset);
  Serial.print(", Y: ");
  Serial.print(accelY_offset);
  Serial.print(", Z: ");
  Serial.println(accelZ_offset);

gyroX_offset_rad = (gyroX_offset / 65.5) * (M_PI / 180.0);
gyroY_offset_rad = (gyroY_offset / 65.5) * (M_PI / 180.0);
gyroZ_offset_rad = (gyroZ_offset / 65.5) * (M_PI / 180.0);
}

void loop(){
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    accelX = accel.acceleration.x - 0.4;
    accelY = accel.acceleration.y + 0.1;
    accelZ = accel.acceleration.z - 0.84;

    Serial.print("Accel X:");
    Serial.print(accelX);
    Serial.print(", Y:");
    Serial.print(accelY);
    Serial.print(", Z:");
    Serial.print(accelZ);
    Serial.print("m/s^2 ");

    gyroX += gyro.gyro.x;
    gyroY += gyro.gyro.y;
    gyroZ += gyro.gyro.z;

    //  Serial.print(gyroX_offset_rad);
    // Serial.print("Gyro X:");
    // Serial.print(degrees(gyroX));
    // Serial.print(", Y:");
    // Serial.print(degrees(gyroY));
    // Serial.print(", Z:");
    // Serial.print(degrees(gyroZ));
    // Serial.println("rad");

     Serial.print(gyroX_offset_rad);
    Serial.print("Gyro X:");
    Serial.print(gyro.gyro.x);
    Serial.print(", Y:");
    Serial.print(gyro.gyro.y);
    Serial.print(", Z:");
    Serial.print(gyro.gyro.z);
    Serial.println("rad");

    // Serial.print(gyroX_offset);
}
