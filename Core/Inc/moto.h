#ifndef __MOTO_H
#define __MOTO_H 

#include "stm32f1xx_hal.h"


extern void moto_contorl(float speed_A, float speed_B);
extern void contorl(void);
extern void contorl_speed(float speed);
extern void contorl_balance(void);

#endif
