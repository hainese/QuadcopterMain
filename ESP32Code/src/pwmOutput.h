#ifndef PWMOUTPUT_H
#define PWMOUTPUT_H

#include <stdint.h>

void pwmInit();
void outputPWM(bool outputEnable, uint8_t ch0, uint8_t ch1, uint8_t ch2, uint8_t ch3);

#endif