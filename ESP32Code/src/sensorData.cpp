#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "Adafruit_VL53L1X.h"
#include "sensorData.h"

float prevAccelXB = 0, prevAccelYB = 0, prevAccelZB = 0;
volatile bool motion_detectedB = false;
const float motionThresholdB = 0.5;

//Adafruit_VL53L1X vl53; // Initialize VL53L1X with custom pins
Adafruit_LSM6DSOX sox; // Create an instance of the LSM6DSOX sensor

void sensorSetup(){
    Wire.begin(8, 9); // Initialize I2C with custom pins

  /*if (!vl53.begin(0x29, &Wire)) {
    Serial.println("Error initializing VL53L1X");
    while (1) delay(10);
  }
  vl53.setTimingBudget(15);
  vl53.startRanging();
  */
  bool temp = 0;

  while(!temp){
    if (!sox.begin_I2C(0x6A)) {
      Serial.println("Failed to find LSM6DSOX chip at address 0x6A");
    }
    else{
      temp =1;
    }
    if (!sox.begin_I2C(0x6B)) {
      Serial.println("Failed to find LSM6DSOX chip at address 0x6B");
    }
    else{
      temp = 1;
    }
    delay(100);
  }


  sox.reset();
  delay(100);
  //sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  //sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
}


void detectMotionB(float accelX, float accelY, float accelZ) {
    if (fabs(accelX - prevAccelXB) > motionThresholdB ||
        fabs(accelY - prevAccelYB) > motionThresholdB ||
        fabs(accelZ - prevAccelZB) > motionThresholdB) {
      motion_detectedB = true;
    }
  
    prevAccelXB = accelX;
    prevAccelYB = accelY;
    prevAccelZB = accelZ;
  }

void collectSensorData(){
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    detectMotionB(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    int16_t gyroXB = (int16_t)(gyro.gyro.x * 1000);
    int16_t gyroYB = (int16_t)(gyro.gyro.y * 1000);
    int16_t gyroZB = (int16_t)(gyro.gyro.z * 1000);

    int16_t distanceB = -1;
    /*if (vl53.dataReady()) {
        distanceB = vl53.distance();
        vl53.clearInterrupt();
    }*/

    //Serial.print(F("Accel (m/s^2): X="));
    //Serial.print(accel.acceleration.x);
   // Serial.print(F(", Y="));
   // Serial.print(accel.acceleration.y);
    //Serial.print(F(", Z="));
   // Serial.println(accel.acceleration.z);

    Serial.print(F("Gyro (mrad/s): X="));
    Serial.print(gyroXB);
    Serial.print(F(", Y="));
    Serial.print(gyroYB);
    Serial.print(F(", Z="));
    Serial.println(gyroZB);

    Serial.print(F("Distance (mm): "));
    Serial.println(distanceB);

    Serial.print(F("Temperature (C): "));
    Serial.println(temp.temperature);

    Serial.print(F("Motion Detected: "));
    Serial.println(motion_detectedB);

    Wire.beginTransmission(0x42);
    Wire.write((uint8_t *)&gyroXB, sizeof(gyroXB));
    Wire.write((uint8_t *)&gyroYB, sizeof(gyroYB));
    Wire.write((uint8_t *)&gyroZB, sizeof(gyroZB));
    //Wire.write((uint8_t *)&distanceB, sizeof(distanceB));
    Wire.write(motion_detectedB ? 1 : 0);
    Wire.endTransmission();

    motion_detectedB = false;
}