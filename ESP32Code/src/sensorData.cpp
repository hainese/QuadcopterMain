#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
//#include "Adafruit_VL53L1X.h"
#include "sensorData.h"

float prevAccelXB = 0, prevAccelYB = 0, prevAccelZB = 0;
volatile bool motion_detectedB = false;
const float motionThresholdB = 0.5;

Adafruit_LSM6DSOX sox; // Create an instance of the LSM6DSOX sensor

void sensorSetup(){
  Wire.begin(8, 9); // Initialize I2C with custom pins
  bool temp = 0;

  while(!temp){
    if (!sox.begin_I2C(0x6A)) {
      Serial.println("Failed to find LSM6DSOX chip at address 0x6A");
    }
    else{
      temp =1;
      Serial.println("Gyro Good");
    }
    delay(100);
  }

  sox.reset();
  delay(100);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
}

void collectSensorData(float *gyroData){
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    int16_t gyroXB = (int16_t)(gyro.gyro.x);
    int16_t gyroYB = (int16_t)(gyro.gyro.y);
    int16_t gyroZB = (int16_t)(gyro.gyro.z);

    gyroData[0] = gyroXB;
    gyroData[1] = gyroYB;
    gyroData[2] = gyroZB;
}