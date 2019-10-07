
#ifndef _BSP_TIMER_H
#define _BSP_TIMER_H

#include "stm8s.h"


void TIMER_Initializes(void);

void TIMDelay_N10us(uint16_t Times);
void TIMDelay_Nms(uint16_t Times);
void TIMDelay_Ns(uint16_t Times);

#endif

