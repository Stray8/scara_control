#ifndef __MotorInit_H
#define __MotorInit_H

#include "stm32f4xx.h"
void MotorIOInit(void);
void MotorNVICInit(void);
void PulseInit(void);
extern void MotorInit(void);
extern void PVD_Init(void);

#endif
