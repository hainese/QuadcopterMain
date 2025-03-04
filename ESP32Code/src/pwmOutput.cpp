#include <Arduino.h>
#include "pwmOutput.h"

const int pwmPin0 = 4;
const int pwmPin1 = 5;
const int pwmPin2 = 38;
const int pwmPin3 = 39;

const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int pwmChannel3 = 3;

const int pwmFrequency = 5000; //Brushed motors work best 1kHz to 20kHz
const int pwmResolution = 8; //2^# bits, so for 8-bit (0-255)

int channel0Duty = 0;
int channel1Duty = 0;
int channel2Duty = 0;
int channel3Duty = 0;


void pwmInit(){
  ledcSetup(pwmChannel0, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel1, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel2, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel3, pwmFrequency, pwmResolution);
  ledcAttachPin(pwmPin0, pwmChannel0);
  ledcAttachPin(pwmPin1, pwmChannel1);
  ledcAttachPin(pwmPin2, pwmChannel2);
  ledcAttachPin(pwmPin3, pwmChannel3);
}

void outputPWM(){
  if(Serial.available() > 0){
    int tempInt = 0;
    String inputString = Serial.readString();
    Serial.println(inputString);
    int commaIndex = inputString.indexOf(',');
    if(commaIndex != -1){
      tempInt = inputString.substring(0, commaIndex).toInt();
      if(tempInt >=0 & tempInt <= 100){
        channel0Duty = tempInt*255/100;
      }
      inputString = inputString.substring(commaIndex +1);
    }
    commaIndex = inputString.indexOf(',');
    if(commaIndex != -1){
      tempInt = inputString.substring(0, commaIndex).toInt();
      if(tempInt >=0 & tempInt <= 100){
        channel1Duty = tempInt*255/100;
      }
      inputString = inputString.substring(commaIndex +1);
    }
    commaIndex = inputString.indexOf(',');
    if(commaIndex != -1){
      tempInt = inputString.substring(0, commaIndex).toInt();
      if(tempInt >=0 & tempInt <= 100){
        channel2Duty = tempInt*255/100;
      }
      inputString = inputString.substring(commaIndex +1);
    }
    tempInt = inputString.toInt();
    if(tempInt >=0 & tempInt <= 100){
      channel3Duty = tempInt*255/100;
    }
    Serial.print("CH1: ");
    Serial.print(channel0Duty);
    Serial.print("; CH2: ");
    Serial.print(channel1Duty);
    Serial.print("; CH3: ");
    Serial.print(channel2Duty);
    Serial.print("; CH4: ");
    Serial.println(channel3Duty);
  }
  ledcWrite(pwmChannel0, channel0Duty);
  ledcWrite(pwmChannel1, channel1Duty);
  ledcWrite(pwmChannel2, channel2Duty);
  ledcWrite(pwmChannel3, channel3Duty);
}