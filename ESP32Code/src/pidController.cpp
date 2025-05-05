
#include <Arduino.h>
#include "pwmOutput.h"
#include "pidController.h"

// PID controller code derived from the following video:
// https://www.youtube.com/watch?v=jY6bBcMtseY

// time between each iteration
// likely needs to be changed based on the rest of the system
//float ts = 0.01;
unsigned long timePrevious_pid = 0;

// PID controller values
// change these values to tune the PID controller based on the system
float p = 0.5; // proportional gain
float i = 0.0; // integral gain
float d = 0.0; // derivative gain

float pangle = 2;
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
    float output = (inputValue * 90);
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
    float output = p * currError + *prevI + i * (currError + *prevError) * ts / 2 + d * (currError - *prevError) / ts; 
    *prevError = currError; // update previous error
    *prevI = i; // update previous integral value
    return output;
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
void pidControl(float *prevRollError, float *prevRollI,
                float *prevPitchError, float *prevPitchI,
                float *prevYawError, float *prevYawI,
                float *prevVerticalVelocityError, float *prevVerticalVelocityI,
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
                float *dutyCycles // array to hold duty cycles for each motor
            ) {

    unsigned long timeCurrent = millis();
    float timeDifference = (timeCurrent - timePrevious_pid) / 1000.0;
    timePrevious_pid = timeCurrent;

    // calculate the desired angles for roll and pitch
    float desiredRollAngle = desiredAngle(rollInput);
    float desiredPitchAngle = desiredAngle(pitchInput);

    // calculate the angle error for roll and pitch
    float rollErrorAngle = errorValueAngle(desiredRollAngle, rollAngle);
    float pitchErrorAngle = errorValueAngle(desiredPitchAngle, pitchAngle);

    // desired rates for roll, pitch, and yaw
    float desiredRollRate = pidEquation(pangle, iangle, dangle, rollErrorAngle, prevRollError, prevRollI, timeDifference);
    float desiredPitchRate = pidEquation(pangle, iangle, dangle, pitchErrorAngle, prevPitchError, prevPitchI, timeDifference);
    float desiredYawRate = desiredRate(yawInput);

    // calculate error values for roll, pitch, and yaw
    float rollError = errorValueRate(desiredRollRate, rollRate);
    float pitchError = errorValueRate(desiredPitchRate, pitchRate);
    float yawError = errorValueRate(desiredYawRate, yawRate);

    float verticalVelocityError = 0 - currentVerticalVelocity;

    // calculate PID input for roll, pitch, and yaw
    float rollPID = pidEquation(p, i, d, rollError, prevRollError, prevRollI, timeDifference);
    float pitchPID = pidEquation(p, i, d, pitchError, prevPitchError, prevPitchI, timeDifference);
    float yawPID = pidEquation(p, i, d, yawError, prevYawError, prevYawI, timeDifference);

    float throttlePID = pidEquation(p, i, d, verticalVelocityError, prevVerticalVelocityError, prevVerticalVelocityI, timeDifference);
    float baseThrottle = throttleInput * 255 + throttlePID;

    // calculate duty cycles for each motor
    dutyCycles[0] = calculateDutyCycleBR(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[1] = calculateDutyCycleFR(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[2] = calculateDutyCycleFL(baseThrottle, rollPID, pitchPID, yawPID);
    dutyCycles[3] = calculateDutyCycleBL(baseThrottle, rollPID, pitchPID, yawPID);

    //find scaling factor
    float scaleFactor = pidScale(dutyCycles[0],dutyCycles[1],dutyCycles[2],dutyCycles[3]);

    //duty cycles  = scale * their duty
    dutyCycles[0] = constrain(scaleFactor * dutyCycles[0], 0, 255);
    dutyCycles[1] = constrain(scaleFactor * dutyCycles[1], 0, 255);
    dutyCycles[2] = constrain(scaleFactor * dutyCycles[2], 0, 255);
    dutyCycles[3] = constrain(scaleFactor * dutyCycles[3], 0, 255);
}