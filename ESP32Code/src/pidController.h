#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

float desiredRate(float inputValue);
float desiredAngle(float inputValue);
float errorValueRate(float desiredRate, float rate);
float errorValueAngle(float desiredAngle, float kalmanAngle);
float pidEquation(float p, float i, float d, float currError, float *prevError, float *prevI, float ts);
float calculateDutyCycleRR(float throttle, float roll, float pitch, float yaw);
float calculateDutyCycleFR(float throttle, float roll, float pitch, float yaw);
float calculateDutyCycleFL(float throttle, float roll, float pitch, float yaw);
float calculateDutyCycleRL(float throttle, float roll, float pitch, float yaw);
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
                float rollAngle,
                float pitchAngle,
                float *dutyCycles
            );

#endif