
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

// Example sensor rates (replace with real gyro data)
float rollRate = 0.0, pitchRate = 0.0, yawRate = 0.0;

float gyroData[3];
float dutyCycles[4];

std::array<uint8_t, 32> packet;
bool buttonA, buttonB, prevButtonA, prevButtonB;
bool globalHeartbeat, expectedHeartbeat;
bool heartbeatEnable, buttonEnable, outputEnable;
uint8_t outputCH0 = 0, outputCH1 = 0, outputCH2 = 0, outputCH3 = 0;

int buttonAEnableTime, buttonBEnableTime,heartbeatTime;

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
  Serial.begin(115200);
  wifiSetup();
  pwmInit();
  heartbeatTime = millis();
  //sensorSetup();
}

void loop() {
  packet = wifiReceive();
  if(packet[0] >= 0b10000000) //valid data check
  {
    buttonA = (packet[0] >> (0)) & 1;
    buttonB = (packet[0] >> (1)) & 1;
    globalHeartbeat = (packet[0] >> (2)) & 1;
    
    outputCH0 = packet[1]; // yaw
    outputCH1 = packet[2]; // throttle
    outputCH2 = packet[3]; // roll
    outputCH3 = packet[4]; // pitch
    
  }

  //collectSensorData();
  
  checkButtonEnable();
  checkButtonDisable();
  checkHeartBeat(globalHeartbeat);
  if(!heartbeatEnable){
    buttonEnable = false;
  }
  outputEnable = heartbeatEnable & buttonEnable;
  

  //outputEnable = 1; // For testing purposes, always enable output

  rollRate = gyroData[0]; // X roll
  pitchRate = gyroData[1]; // Y pitch
  yawRate = gyroData[2]; // Z yaw
  
  // calculate duty cycles
  pidControl(
    &prevRollError, &prevRollI,
    &prevPitchError, &prevPitchI,
    &prevYawError, &prevYawI,
    outputCH2, 
    outputCH3, 
    outputCH0, 
    outputCH1, 
    rollRate, 
    pitchRate, 
    yawRate,
    dutyCycles
  );

  // testing purposes only
  /*
  Serial.println((String)"m1: "+dutyCycles[0]+
                 (String)"\nm2: "+dutyCycles[1]+
                 (String)"\nm3: "+dutyCycles[2]+
                 (String)"\nm4: "+dutyCycles[3]);
  */

  // output duty cycles to motors
  outputPWM(outputEnable, dutyCycles[0], dutyCycles[1], dutyCycles[2], dutyCycles[3]);
}
