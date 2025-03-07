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

void outputPWM(bool outputEnable, uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3){
  channel0Duty = ch0*255/100;
  channel1Duty = ch1*255/100;
  channel2Duty = ch2*255/100;
  channel3Duty = ch3*255/100;
  
  if(outputEnable){
    ledcWrite(pwmChannel0, channel0Duty);
    ledcWrite(pwmChannel1, channel1Duty);
    ledcWrite(pwmChannel2, channel2Duty);
    ledcWrite(pwmChannel3, channel3Duty);
  }
  else{
    ledcWrite(pwmChannel0, 0);
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, 0);
  }
}