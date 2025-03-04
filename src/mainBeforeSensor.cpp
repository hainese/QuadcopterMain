/*#include <Arduino.h>
#include "wirelessConnection.h"
#include "pwmOutput.h"

int stopVal = 0;
int startVal = 0;
int sensorGyroX = 0;
int sensorGyroY = 0;
int sensorGyroZ = 0;
int userInputX = 0;
int userInputY = 0;
int userInputZ = 0;
int sensorTof = 0;
int wifiHeartbeat = 0;
int count = 0;

void outputToSoftware(){
  Serial.print("stopVal: ");
  Serial.print(stopVal);
  Serial.print(", startVal: ");
  Serial.print(startVal);
  Serial.print(", sensorGyroX: ");
  Serial.print(sensorGyroX);
  Serial.print(", sensorGyroY: ");
  Serial.print(sensorGyroY);
  Serial.print(", sensorGyroZ: ");
  Serial.print(sensorGyroZ);
  Serial.print(", userInputX: ");
  Serial.print(userInputX);
  Serial.print(", userInputY: ");
  Serial.print(userInputY);
  Serial.print(", userInputZ: ");
  Serial.print(userInputZ);
  Serial.print(", sensorTof: ");
  Serial.print(sensorTof);
  Serial.print(", wifiHeartbeat: ");
  Serial.println(wifiHeartbeat);
  count++;
  if(count > 255) count = 0;
  stopVal = count%2;
  startVal = count%2;
  sensorGyroX = count;
  sensorGyroY = count;
  sensorGyroZ = count;
  userInputX = count;
  userInputY = count;
  userInputZ = count;
  sensorTof = count*(2970)/255+30;
  wifiHeartbeat = count%2;
}

void setup(){
  Serial.begin(115200);
  wifiSetup();
  pwmInit();
}

void loop(){
  wifiReceive();
  outputPWM();
  outputToSoftware();
  delay(500);
}
*/