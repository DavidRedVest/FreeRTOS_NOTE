#ifndef __TIM_H__
#define __TIM_H__

#include "main.h"

extern TIM_HandleTypeDef htim6;

void tim6_init(void);
void HAL_Delay(uint32_t Delay);
#endif
