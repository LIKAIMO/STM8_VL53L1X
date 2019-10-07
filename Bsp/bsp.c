
#include "bsp.h"
#include "bsp_timer.h"
#include "bsp_i2c.h"
#include "vl53l1x.h"

void sysInit(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); //内部时钟16M
	CLK_HSICmd(ENABLE);
	UART1->BRR2 = 0x0A;
	UART1->BRR1 = 0x08;
	UART1->CR1 = 0x00;
	UART1->CR2 = 0x2C;
	UART1->SR = 0xC0;
	enableInterrupts();
	TIMER_Initializes();
	I2C_Initializes();
}

void dataHandle(void)
{

	if (VL53L1X_Init())
	{
		//init fail
		return;
	}

	u16 distance = 0x0000;
	u8 *data = (u8 *)&distance;

	while (1)
	{
		distance = VL53L1X_MEASURE();
		for (u8 i = 0; i < 2; i++)
		{
			while ((UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET))
				;
			UART1->DR = data[i];
		}

		TIMDelay_Nms(80);
	}
}

#ifdef USE_FULL_ASSERT

void assert_failed(u8 *file, u32 line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif