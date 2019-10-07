
#include "bsp_timer.h"

void TIMER_Initializes(void)
{
	TIM4_TimeBaseInit(TIM4_PRESCALER_2, 79);
	TIM4_ClearFlag(TIM4_FLAG_UPDATE);
}

void TIMDelay_N10us(uint16_t Times)
{
	TIM4_Cmd(ENABLE);
	while (Times--)
	{
		while (RESET == TIM4_GetFlagStatus(TIM4_FLAG_UPDATE))
			;
		TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	}
	TIM4_Cmd(DISABLE);
}

void TIMDelay_Nms(uint16_t Times)
{
	while (Times--)
	{
		TIMDelay_N10us(100);
	}
}

void TIMDelay_Ns(uint16_t Times)
{
	while (Times--)
	{
		TIMDelay_Nms(1000);
	}
}
