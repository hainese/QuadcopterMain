
#include "pwmOutput.h"

// PID controller code derived from the following video:
// https://www.youtube.com/watch?v=jY6bBcMtseY
float ts = 0.01;

// calculate desired rates for roll, pitch, and yaw
float desiredRate(float inputValue){
    float output = 0.15 * (inputValue-1500);
    return output;
}

float inputThrottle(float inputValue){
    float output = inputValue;
    return output;
}

// calculate error values for roll, pitch, and yaw
float errorValue(float desiredRate, float rate) {
    float output = desiredRate - rate;
    return output;
}

// calculate PID input for roll, pitch, and yaw
float calculatePIDinput(float p, float i, float d, float currError, float *prevError, float *prevI){
    float output = p * currError + *prevI + i * (currError + *prevError) * ts / 2 + d * (currError - *prevError) / ts; 
    *prevError = currError;
    *prevI = i;
    return output;
}

int calculateDutyCycle1(float throttle, float roll, float pitch, float yaw){
    int output = throttle - pitch - roll - yaw;
    return output;
}

int calculateDutyCycle2(float throttle, float roll, float pitch, float yaw){
    int output = throttle + pitch - roll + yaw;
    return output;
}

int calculateDutyCycle3(float throttle, float roll, float pitch, float yaw){
    int output = throttle + pitch + roll - yaw;
    return output;
}

int calculateDutyCycle4(float throttle, float roll, float pitch, float yaw){
    int output = throttle - pitch + roll + yaw;
    return output;
}

void pidControl(float *prevRollError, float *prevRollI,
                float *prevPitchError, float *prevPitchI,
                float *prevYawError, float *prevYawI,
                float rollInput, 
                float pitchInput, 
                float yawInput, 
                float throttleInput, 
                float rollRate, 
                float pitchRate, 
                float yawRate) {
    // PID controller values
    float p = 0.1; // proportional gain
    float i = 0.01; // integral gain
    float d = 0.01; // derivative gain

    // desired rates for roll, pitch, and yaw
    float desiredRollRate = desiredRate(rollInput);
    float desiredPitchRate = desiredRate(pitchInput);
    float desiredYawRate = desiredRate(yawInput);

    // throttle input
    float throttle = inputThrottle(throttleInput);

    // calculate error values for roll, pitch, and yaw
    float rollError = errorValue(desiredRollRate, rollRate);
    float pitchError = errorValue(desiredPitchRate, pitchRate);
    float yawError = errorValue(desiredYawRate, yawRate);

    // calculate PID input for roll, pitch, and yaw
    float rollPID = calculatePIDinput(p, i, d, rollError, prevRollError, prevRollI);
    float pitchPID = calculatePIDinput(p, i, d, pitchError, prevPitchError, prevPitchI);
    float yawPID = calculatePIDinput(p, i, d, yawError, prevYawError, prevYawI);

    // calculate duty cycles for each motor
    int dutyCycle1 = calculateDutyCycle1(throttle, rollPID, pitchPID, yawPID);
    int dutyCycle2 = calculateDutyCycle2(throttle, rollPID, pitchPID, yawPID);
    int dutyCycle3 = calculateDutyCycle3(throttle, rollPID, pitchPID, yawPID);
    int dutyCycle4 = calculateDutyCycle4(throttle, rollPID, pitchPID, yawPID);

    // output PWM signals to motors
    outputPWM(true,dutyCycle1,dutyCycle2,dutyCycle3,dutyCycle4);
}