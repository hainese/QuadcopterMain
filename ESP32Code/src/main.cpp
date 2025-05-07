
#include <Arduino.h>
#include <Wire.h>
#include "wirelessConnection.h"
#include "pwmOutput.h"
#include "sensorData.h"
#include "pidController.h"

//#include <ctime>

// Example sensor rates (replace with real gyro data)
float rollRate = 0.0, pitchRate = 0.0, yawRate = 0.0;
float rollAngle = 0.0, pitchAngle = 0.0;
float verticalVel;

float accelData[3];
float angleData[2];
float gyroData[3];
float velData[1];
float dutyCycles[4];
uint8_t dutyCycle8bit[4];
float maxGyro = 0;

std::array<uint8_t, 32> packet;
bool buttonA, buttonB, prevButtonA, prevButtonB;
bool globalHeartbeat, expectedHeartbeat;
bool heartbeatEnable, buttonEnable, outputEnable;
float userThrottle = 0, userYaw = 0, userPitch = 0, userRoll = 0;

int buttonAEnableTime, buttonBEnableTime, heartbeatTime;

void checkHeartBeat(bool heartbeat){
  if(millis() - heartbeatTime > 3100){
    //Serial.println("HEARTBEAT LOST");
    heartbeatEnable = false;
  }
  if(heartbeat == expectedHeartbeat){
      expectedHeartbeat = !expectedHeartbeat;
      heartbeatTime = millis();
      heartbeatEnable = true;
  }
}

void checkButtonEnable(){
  if(!prevButtonA && buttonA){
    buttonAEnableTime = millis();
  }
  if(buttonA){
    if(millis()-buttonAEnableTime > 3500){
      //Serial.println("Enabling Outputs");
      buttonEnable = 1;
    }
  }
  prevButtonA = buttonA;
}

void checkButtonDisable(){
  if(!prevButtonB && buttonB){
    buttonBEnableTime = millis();
  }
  if(buttonB){
    if(millis()-buttonBEnableTime > 100){
      //Serial.println("Disabling Outputs");
      buttonEnable = 0;
    }
  }
  prevButtonB = buttonB;
}

void setup() {
  Serial.begin(115200);
  wifiSetup();
  pwmInit();
  heartbeatTime = millis();
  sensorSetup();
}

void loop() {
  packet = wifiReceive();
  if(packet[0] >= 0b10000000) //valid data check
  {
    buttonA = (packet[0] >> (0)) & 1;
    buttonB = (packet[0] >> (1)) & 1;
    globalHeartbeat = (packet[0] >> (2)) & 1;
    
    uint32_t packetCombiner = ((packet[1]<<24) | (packet[2]<<16) | (packet[3]<<8) | packet [4]);
    memcpy(&userYaw, &packetCombiner, sizeof(packetCombiner));
    packetCombiner = ((packet[5]<<24) | (packet[6]<<16) | (packet[7]<<8) | packet [8]);
    memcpy(&userThrottle, &packetCombiner, sizeof(packetCombiner));
    packetCombiner = ((packet[9]<<24) | (packet[10]<<16) | (packet[11]<<8) | packet [12]);
    memcpy(&userRoll, &packetCombiner, sizeof(packetCombiner));
    packetCombiner = ((packet[13]<<24) | (packet[14]<<16) | (packet[15]<<8) | packet [16]);
    memcpy(&userPitch, &packetCombiner, sizeof(packetCombiner));
  }

  /*Serial.print("User Roll: " + (String)userRoll);
  Serial.print(" User Pitch: " + (String)userPitch);
  Serial.print(" User Yaw: " + (String)userYaw);
  Serial.print(" User Throttle: " + (String)userThrottle);
  Serial.println();*/

  
  collectSensorData(accelData, gyroData, angleData, velData);
  checkButtonEnable();
  checkButtonDisable();
  checkHeartBeat(globalHeartbeat);
  if(!heartbeatEnable){
    buttonEnable = false;
  }
  outputEnable = heartbeatEnable & buttonEnable;
  

  //outputEnable = 1; // For testing purposes, always enable output

  //accelData[0] = accel.acceleration.x;
  //accelData[1] = accel.acceleration.y;
  //accelData[2] = accel.acceleration.z;
  rollAngle = angleData[0]; // roll angle
  pitchAngle = angleData[1]; // pitch angle
  rollRate = gyroData[0]; // X roll
  pitchRate = gyroData[1]; // Y pitch
  yawRate = gyroData[2]; // Z yaw
  verticalVel = velData[0];
  


  // calculate duty cycles
  pidControl(
    userRoll, 
    userPitch, 
    userYaw, 
    userThrottle, 
    rollRate, 
    pitchRate, 
    yawRate,
    rollAngle,
    pitchAngle,
    verticalVel,
    dutyCycles,
    outputEnable
  );

  dutyCycle8bit[0] = (uint8_t)dutyCycles[0];
  dutyCycle8bit[1] = (uint8_t)dutyCycles[1];
  dutyCycle8bit[2] = (uint8_t)dutyCycles[2];
  dutyCycle8bit[3] = (uint8_t)dutyCycles[3];

  // testing purposes only
  
  /*Serial.println(
    (String)"verticalVel: "+verticalVel+
    (String)"userThrottle: "+userThrottle+
    (String)"roll: "+userRoll+
    (String)"pitch "+userPitch+
    (String)"yaw: "+userYaw+
    (String)"m1: "+dutyCycle8bit[0]+
    (String)"m2: "+dutyCycle8bit[1]+
    (String)"m3: "+dutyCycle8bit[2]+
    (String)"m4: "+dutyCycle8bit[3]);*/
    
                 
  // output duty cycles to motors
  outputPWM(outputEnable, dutyCycle8bit[0], dutyCycle8bit[1], dutyCycle8bit[2], dutyCycle8bit[3]);
}
