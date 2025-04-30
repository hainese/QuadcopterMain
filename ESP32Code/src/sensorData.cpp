#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "sensorData.h"

float pitchAct = 0;
float rollAct = 0;

unsigned long timePrevious = 0;


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
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
}

void collectSensorData(float *accelData, float *gyroData){
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  unsigned long timeCurrent = millis();
  float timeDifference = (timeCurrent - timePrevious) / 1000.0;
  timePrevious = timeCurrent;

  float gyroX = gyro.gyro.x * 180/PI;
  float gyroY = gyro.gyro.y * 180/PI;
  float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z)*180/PI;
  float accelPitch = atan2(-accel.acceleration.x, sqrt(pow(accel.acceleration.y,2) + pow(accel.acceleration.z,2)))*180/PI;

  const float p = .96;
  rollAct = p * (rollAct + gyroX*timeDifference) + (1-p)*accelRoll;
  pitchAct = p * (pitchAct + gyroY*timeDifference) + (1-p)*accelPitch;

  Serial.print("Accel R: " + (String)accelRoll + ", ");
  Serial.print("Accel P: " + (String)accelPitch + ", ");
  Serial.print("Gyro X: " + (String)gyro.gyro.x + ", ");
  Serial.print("Gyro Y: " + (String)gyro.gyro.y + ", ");
  Serial.print("RollA: " + (String)rollAct + ", ");
  Serial.print("PitchA: " + (String)pitchAct + ", ");
  Serial.println();

  accelData[0] = accel.acceleration.x;
  accelData[1] = accel.acceleration.y;
  accelData[2] = accel.acceleration.z;
  gyroData[0] = rollAct;
  gyroData[1] = pitchAct;
  gyroData[2] = gyro.gyro.z;
}