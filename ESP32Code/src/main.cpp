#include <Arduino.h>
#include <Wire.h>
#include "wirelessConnection.h"
#include "pwmOutput.h"
#include "sensorData.h"

//#include <ctime>

std::array<uint8_t, 32> packet;
bool buttonA, buttonB, prevButtonA, prevButtonB;
bool globalHeartbeat, expectedHeartbeat;
bool heartbeatEnable, buttonEnable, outputEnable;
uint8_t outputCH0 = 0, outputCH1 = 0, outputCH2 = 0, outputCH3 = 0;

int buttonAEnableTime, buttonBEnableTime,heartbeatTime;

void checkHeartBeat(bool heartbeat){
  if(millis() - heartbeatTime > 3100){
    Serial.println("HEARTBEAT LOST");
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
      Serial.println("Enabling Outputs");
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
      Serial.println("Disabling Outputs");
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
    
    outputCH0 = packet[1];
    outputCH1 = packet[2];
    outputCH2 = packet[3];
    outputCH3 = 0;
    
  }

  //collectSensorData();
  checkButtonEnable();
  checkButtonDisable();
  checkHeartBeat(globalHeartbeat);
  if(!heartbeatEnable){
    buttonEnable = false;
  }
  outputEnable = heartbeatEnable & buttonEnable;
  outputPWM(outputEnable,outputCH0, outputCH1, outputCH2, outputCH3);

}
