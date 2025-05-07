
#include <Arduino.h>
#include "pwmOutput.h"
#include "pidController.h"

// pid init variables
float prevRollError = 0, prevRollI = 0;
float prevPitchError = 0, prevPitchI = 0;
float prevYawError = 0, prevYawI = 0;
float prevVerticalVelocityError = 0, prevVerticalVelocityI = 0;

// PID controller code derived from the following video:
// https://www.youtube.com/watch?v=jY6bBcMtseY

// time between each iteration
// likely needs to be changed based on the rest of the system
//float ts = 0.01;
unsigned long timePreviousPid = 0;

// PID controller values
// change these values to tune the PID controller based on the system

float pRate = 2.75; // proportional gain
float iRate = .2; // integral gain
float dRate =  0.1; // derivative gain

float pRateYaw = 0;
float iRateYaw = 0;
float dRateYaw = 0;

float pangle = 1;
float iangle = 0;
float dangle = 0;

// calculate desired rates for roll, pitch, and yaw
// may need to change
float desiredRate(float inputValue){
    //float output = 0.15 * (inputValue-1500);
    //float output = inputValue - 127.5; // map input value from 0-255 to -127.5 to 127.5
    float output = inputValue;
    return output;
}

// calculate desired angles for roll, pitch, and yaw from -90 to 90 degrees
float desiredAngle(float inputValue){
    //float output = ((inputValue - 127.5) / 127.5) * 90; // -1 and 1 to -90 and 90
    float output = (inputValue * 10);
    return output;
}

// calculate error values for roll, pitch, and yaw rates
float errorValueRate(float desiredRate, float rate) {
    return desiredRate - rate;
}

// calculate error values for roll, pitch, and yaw angles
float errorValueAngle(float desiredAngle, float kalmanAngle) {
    return desiredAngle - kalmanAngle;
}

// calculate PID for roll, pitch, and yaw
float pidEquation(float p, float i, float d, float currError, float *prevError, float *prevI, float ts){
    // PID control equation:
    float pTerm = p * currError;
    float iTerm = *prevI + (i * (currError + *prevError)*ts) /2;
    iTerm = (iTerm > 255) ? 255 : ((iTerm < -255) ? -255 : iTerm);
    float dTerm = d * (currError-*prevError) / ts;
    dTerm = (dTerm > 255) ? 255 : ((dTerm < -255) ? -255 : dTerm);
    float output = pTerm + iTerm + dTerm;
    
    *prevError = currError;
    *prevI = iTerm;
    return output;
}

void pidReset(){
    prevRollError = 0; prevRollI = 0;
    prevPitchError = 0; prevPitchI = 0;
    prevYawError = 0; prevYawI = 0;
    prevVerticalVelocityError = 0; prevVerticalVelocityI = 0;
}

// Duty Cycle for motor Front Left
float calculateDutyCycleFL(float throttle, float roll, float pitch, float yaw){
    return throttle - pitch + roll - yaw;
}

// Duty Cycle for motor Front Right
float calculateDutyCycleFR(float throttle, float roll, float pitch, float yaw){
    return throttle - pitch - roll + yaw;
}

// Duty Cycle for motor Rear Right
float calculateDutyCycleBR(float throttle, float roll, float pitch, float yaw){
    return throttle + pitch - roll - yaw;
}

// Duty Cycle for motor RL
float calculateDutyCycleBL(float throttle, float roll, float pitch, float yaw){
    return throttle + pitch + roll + yaw;
}

// Scaling PID Output
float pidScale(float duty0, float duty1, float duty2, float duty3) {
    //find max duty
    float maxDuty = max(max(duty0,duty1),max(duty2,duty3));
    //find scale factor
    float scaleFactor = 1.0;
    if(maxDuty/255 > 1.0)
    {
        scaleFactor = 1.0 / (maxDuty/255);
    }
    return scaleFactor;
}

// PID control block
void pidControl(
    float rollInput, // user input for roll
    float pitchInput, // user input for pitch
    float yawInput, // user input for yaw
    float throttleInput, // user input for throttle
    float rollRate, // gyro input for roll
    float pitchRate, // gyro input for pitch
    float yawRate, // gyro input for yaw
    float rollAngle, // angle input for roll
    float pitchAngle, // angle input for pitch
    float currentVerticalVelocity, // vertical velocity input
    float *dutyCycles, // array to hold duty cycles for each motor
    bool enable
) {

    unsigned long timeCurrent = millis();
    float timeDifference = (timeCurrent - timePreviousPid) / 1000.0;
    timePreviousPid = timeCurrent;

    // calculate the desired angles for roll and pitch
    float desiredRollAngle = desiredAngle(rollInput);
    float desiredPitchAngle = desiredAngle(pitchInput);

    // calculate the angle error for roll and pitch
    float rollErrorAngle = errorValueAngle(desiredRollAngle, rollAngle);
    float pitchErrorAngle = errorValueAngle(desiredPitchAngle, pitchAngle);

    // desired rates for roll, pitch, and yaw
    float desiredRollRate = pidEquation(pangle, iangle, dangle, rollErrorAngle, &prevRollError, &prevRollI, timeDifference);
    float desiredPitchRate = pidEquation(pangle, iangle, dangle, pitchErrorAngle, &prevPitchError, &prevPitchI, timeDifference);
    float desiredYawRate = desiredRate(yawInput);

    // calculate error values for roll, pitch, and yaw
    float rollRateError = errorValueRate(desiredRollRate, rollRate);
    float pitchRateError = errorValueRate(desiredPitchRate, pitchRate);
    float yawRateError = errorValueRate(desiredYawRate, yawRate);

    float verticalVelocityError = 0 - currentVerticalVelocity;

    // calculate PID input for roll, pitch, and yaw
    float rollPID = pidEquation(pRate, iRate, dRate, rollRateError, &prevRollError, &prevRollI, timeDifference);
    float pitchPID = pidEquation(pRate, iRate, dRate, pitchRateError, &prevPitchError, &prevPitchI, timeDifference);
    float yawPID = pidEquation(pRateYaw, iRateYaw, dRateYaw, yawRateError, &prevYawError, &prevYawI, timeDifference);
    //float yawPID = 0;

    /*Serial.print("RollPID");
    Serial.print(rollPID, 4);
    Serial.print(" PitchPID: ");
    Serial.print(pitchPID, 4);
    Serial.print(" yawPId: " + (String)yawPID);
    Serial.print(" Throttle: " + (String)throttleInput);
    Serial.println();*/

    //float throttlePID = pidEquation(p, i, d, verticalVelocityError, prevVerticalVelocityError, prevVerticalVelocityI, timeDifference);
    float throttlePID = 0;
    float baseThrottle = throttleInput * 255 + throttlePID;


    // calculate duty cycles for each motor
    dutyCycles[0] = calculateDutyCycleBR(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[1] = calculateDutyCycleFR(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[2] = calculateDutyCycleFL(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[3] = calculateDutyCycleBL(baseThrottle, rollPID, pitchPID, yawPID);

    Serial.print(dutyCycles[3]);
    Serial.println();
    
    //find scaling factor
    //float scaleFactor = pidScale(dutyCycles[0],dutyCycles[1],dutyCycles[2],dutyCycles[3]);
    float scaleFactor = 1;

    //duty cycles  = scale * their duty
    dutyCycles[0] = constrain(scaleFactor * dutyCycles[0], 0, 255);
    dutyCycles[1] = constrain(scaleFactor * dutyCycles[1], 0, 255);
    dutyCycles[2] = constrain(scaleFactor * dutyCycles[2], 0, 255);
    dutyCycles[3] = constrain(scaleFactor * dutyCycles[3], 0, 255);

    if(!enable || throttleInput < .3){
        pidReset();
    }
}