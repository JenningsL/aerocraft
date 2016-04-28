#ifndef __MOTO_H
#define __MOTO_H

#include <stm32f4_discovery.h>
#include "PID.h"
#include "SysTick.h"

void TIM4_Config(void);
void TIM4_GPIO_Config();

void TIM2_Config(void);
void TIM2_NVIC_Configuare(void);

void setMotos(int moto1, int moto2, int moto3, int moto4);
void control(float pitch, float roll, float pitch_set, float roll_set);
void setThrottle(int t);
int getThrottle();
void unLock(void);
void lock(void);
int isLocked();




#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

#endif