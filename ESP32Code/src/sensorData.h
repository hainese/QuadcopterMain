#ifndef SENSORDATA_H
#define SENSORDATA_H

void sensorSetup();
void detectMotionB(float accelX, float accelY, float accelZ);
void collectSensorData(float *gyroData);

#endif