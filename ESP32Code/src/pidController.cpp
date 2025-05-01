
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
float p = 0.1; // proportional gain
float i = 0.01; // integral gain
float d = 0.01; // derivative gain

float pangle = 2;
float iangle = 0;
float dangle = 0;

// calculate desired rates for roll, pitch, and yaw
// may need to change

float desiredYRate(float inputValue){
    //float output = 0.15 * (inputValue-1500);
    float output = inputValue - 127.5; // map input value from 0-255 to -127.5 to 127.5
    //float output = inputValue;
    return output;
}
float desiredRate(float p, float i, float d, float currError, float *prevError, float *prevI, float ts){
    float output = p * currError + *prevI + i * (currError + *prevError) * ts / 2 + d * (currError - *prevError) / ts;
    *prevError = currError; // update previous error
    *prevI = i; // update previous integral value
    return output;
}

float desiredAngle(float inputValue){
    float output = inputValue * 90; // -1 and 1 to -90 and 90
    return output;
}

// calculate error values for roll, pitch, and yaw
float errorValueRate(float desiredRate, float rate) {
    return desiredRate - rate;
}

float errorValueAngle(float desiredAngle, float kalmanAngle) {
    return desiredAngle - kalmanAngle;
}

// calculate PID input for roll, pitch, and yaw
float calculatePIDinput(float p, float i, float d, float currError, float *prevError, float *prevI, float ts){
    // PID control equation:
    float output = p * currError + *prevI + i * (currError + *prevError) * ts / 2 + d * (currError - *prevError) / ts; 
    *prevError = currError; // update previous error
    *prevI = i; // update previous integral value
    return output;
}

// limits output to 0-100% duty cycle
// likely needs to be changed if the output is consistently above 100% or below 0%
/*
float saturateDutyCycle(float value){
    if(value > 100){
        return 100;
    } else if(value < 0){
        return 0;
    } else {
        return value;
    }
}
*/

// Duty Cycle for motor Front Left
float calculateDutyCycleFL(float throttle, float roll, float pitch, float yaw){
    return throttle - pitch - roll;// - yaw;
}

// Duty Cycle for motor Front Right
float calculateDutyCycleFR(float throttle, float roll, float pitch, float yaw){
    return throttle + pitch - roll;// + yaw;
}

// Duty Cycle for motor Rear Right
float calculateDutyCycleRR(float throttle, float roll, float pitch, float yaw){
    return throttle + pitch + roll;// - yaw;
}

// Duty Cycle for motor RL
float calculateDutyCycleRL(float throttle, float roll, float pitch, float yaw){
    return throttle - pitch + roll;// + yaw;
}

// PID control block
void pidControl(float *prevRollError, float *prevRollI,
                float *prevPitchError, float *prevPitchI,
                float *prevYawError, float *prevYawI,
                float rollInput, // user input for roll
                float pitchInput, // user input for pitch
                float yawInput, // user input for yaw
                float throttleInput, // user input for throttle
                float rollRate, // gyro input for roll
                float pitchRate, // gyro input for pitch
                float yawRate, // gyro input for yaw
                float rollAngle, // angle input for roll
                float pitchAngle, // angle input for pitch
                float *dutyCycles // array to hold duty cycles for each motor
            ) {

    unsigned long timeCurrent = millis();
    float timeDifference = (timeCurrent - timePrevious_pid) / 1000.0;
    timePrevious_pid = timeCurrent;

    float desiredRollAngle = desiredAngle(rollInput);
    float desiredPitchAngle = desiredAngle(pitchInput);

    float rollErrorAngle = errorValueAngle(desiredRollAngle, rollAngle);
    float pitchErrorAngle = errorValueAngle(desiredPitchAngle, pitchAngle);

    // desired rates for roll, pitch, and yaw
    float desiredRollRate = desiredRate(pangle, iangle, dangle, rollErrorAngle, prevRollError, prevRollI, timeDifference);
    float desiredPitchRate = desiredRate(pangle, iangle, dangle, pitchErrorAngle, prevPitchError, prevPitchI, timeDifference);
    float desiredYawRate = desiredYRate(yawInput);

    // calculate error values for roll, pitch, and yaw
    float rollError = errorValueRate(desiredRollRate, rollRate);
    float pitchError = errorValueRate(desiredPitchRate, pitchRate);
    float yawError = errorValueRate(desiredYawRate, yawRate);

    // calculate PID input for roll, pitch, and yaw
    float rollPID = calculatePIDinput(p, i, d, rollError, prevRollError, prevRollI, timeDifference);
    float pitchPID = calculatePIDinput(p, i, d, pitchError, prevPitchError, prevPitchI, timeDifference);
    float yawPID = calculatePIDinput(p, i, d, yawError, prevYawError, prevYawI, timeDifference);

    // calculate duty cycles for each motor
    dutyCycles[0] = calculateDutyCycleRR(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[1] = calculateDutyCycleFR(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[2] = calculateDutyCycleFL(throttleInput, rollPID, pitchPID, yawPID);
    dutyCycles[3] = calculateDutyCycleRL(throttleInput, rollPID, pitchPID, yawPID);
}