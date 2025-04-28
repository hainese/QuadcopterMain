
#include <Arduino.h>
#include "pwmOutput.h"
#include "pidController.h"

// PID controller code derived from the following video:
// https://www.youtube.com/watch?v=jY6bBcMtseY

float ts = 0.01;

// PID controller values
float p = 0.1; // proportional gain
float i = 0.01; // integral gain
float d = 0.01; // derivative gain

// calculate desired rates for roll, pitch, and yaw
float desiredRate(float inputValue){
    //float output = 0.15 * (inputValue-1500);
    //float output = inputValue - 127.5; // map input value from 0-255 to -127.5 to 127.5
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

float saturateDutyCycle(float value){
    if(value > 100){
        return 100;
    } else if(value < 0){
        return 0;
    } else {
        return value;
    }
}

// Duty Cycle for motor 1
float calculateDutyCycle1(float throttle, float roll, float pitch, float yaw){
    float output = saturateDutyCycle(throttle - pitch - roll - yaw);
    return output;
}

// Duty Cycle for motor 2
float calculateDutyCycle2(float throttle, float roll, float pitch, float yaw){
    float output = saturateDutyCycle(throttle + pitch - roll + yaw);
    return output;
}

// Duty Cycle for motor 3
float calculateDutyCycle3(float throttle, float roll, float pitch, float yaw){
    float output = saturateDutyCycle(throttle + pitch + roll - yaw);
    return output;
}

// Duty Cycle for motor 4
float calculateDutyCycle4(float throttle, float roll, float pitch, float yaw){
    float output = saturateDutyCycle(throttle - pitch + roll + yaw);
    return output;
}

// PID control block
void pidControl(float *prevRollError, float *prevRollI,
                float *prevPitchError, float *prevPitchI,
                float *prevYawError, float *prevYawI,
                float rollInput, 
                float pitchInput, 
                float yawInput, 
                float throttleInput, 
                float rollRate, 
                float pitchRate, 
                float yawRate,
                float *dutyCycles
            ) {

    // desired rates for roll, pitch, and yaw
    float desiredRollRate = desiredRate(rollInput);
    float desiredPitchRate = desiredRate(pitchInput);
    float desiredYawRate = desiredRate(yawInput);

    // calculate error values for roll, pitch, and yaw
    float rollError = errorValue(desiredRollRate, rollRate);
    float pitchError = errorValue(desiredPitchRate, pitchRate);
    float yawError = errorValue(desiredYawRate, yawRate);

    // calculate PID input for roll, pitch, and yaw
    float rollPID = calculatePIDinput(p, i, d, rollError, prevRollError, prevRollI);
    float pitchPID = calculatePIDinput(p, i, d, pitchError, prevPitchError, prevPitchI);
    float yawPID = calculatePIDinput(p, i, d, yawError, prevYawError, prevYawI);

    // calculate duty cycles for each motor
    dutyCycles[0] = calculateDutyCycle1(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[1] = calculateDutyCycle2(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[2] = calculateDutyCycle3(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[3] = calculateDutyCycle4(throttleInput, rollPID, pitchPID, yawPID);
}