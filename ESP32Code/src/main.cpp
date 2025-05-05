
#include <Arduino.h>
#include <Wire.h>
#include "wirelessConnection.h"
#include "pwmOutput.h"
#include "sensorData.h"
#include "pidController.h"

//#include <ctime>

// pid init variables
float prevRollError = 0, prevRollI = 0;
float prevPitchError = 0, prevPitchI = 0;
float prevYawError = 0, prevYawI = 0;
float prevVerticalVelocityError = 0, prevVerticalVelocityI = 0;

// Example sensor rates (replace with real gyro data)
float rollRate = 0.0, pitchRate = 0.0, yawRate = 0.0;
float rollAngle = 0.0, pitchAngle = 0.0;
float verticalVel;

float accelData[3];
float angleData[2];
float gyroData[3];
float velData[1];
float dutyCycles[4];
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
    if(millis()-buttonBEnableTime > 1000){
      //Serial.println("Disabling Outputs");
      buttonEnable = 0;
    }
  }
  prevButtonB = buttonB;
}

void setup() {
  //Serial.begin(115200);
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
    &prevRollError, &prevRollI,
    &prevPitchError, &prevPitchI,
    &prevYawError, &prevYawI,
    &prevVerticalVelocityError, &prevVerticalVelocityI,
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
    dutyCycles
  );

  // testing purposes only
  /*
  /Serial.println(
    (String)"verticalVel: "+verticalVel+
    (String)"\tuserThrottle: "+userThrottle+
    (String)"\troll: "+userRoll+
    (String)"\tpitch "+userPitch+
    (String)"\tyaw: "+userYaw+
    (String)"\tm1: "+dutyCycles[0]+
    (String)"\tm2: "+dutyCycles[1]+
    (String)"\tm3: "+dutyCycles[2]+
    (String)"\tm4: "+dutyCycles[3]);
    */
                 
  // output duty cycles to motors
  outputPWM(outputEnable, dutyCycles[0], dutyCycles[1], dutyCycles[2], dutyCycles[3]);
}
