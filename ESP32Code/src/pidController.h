#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

float desiredRate(float inputValue);
float inputThrottle(float inputValue);
float errorValue(float desiredRate, float rate);
float calculatePIDinput(float p, float i, float d, float currError, float *prevError, float *prevI);
int calculateDutyCycle1(float throttle, float roll, float pitch, float yaw);
int calculateDutyCycle2(float throttle, float roll, float pitch, float yaw);
int calculateDutyCycle3(float throttle, float roll, float pitch, float yaw);
int calculateDutyCycle4(float throttle, float roll, float pitch, float yaw);
void pidControl(float *prevRollError, float *prevRollI,
                float *prevPitchError, float *prevPitchI,
                float *prevYawError, float *prevYawI,
                float rollInput, 
                float pitchInput, 
                float yawInput, 
                float throttleInput, 
                float rollRate, 
                float pitchRate, 
                float yawRate);

#endif