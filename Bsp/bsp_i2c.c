#include "bsp_i2c.h"
#include "bsp_timer.h"

#define SET_W I2C->DR = 0x52 //0x29 << 1
#define SET_R I2C->DR = 0x53 //0x29 << 1 + 1

uint8_t i2cBuf[4];

void I2C_Initializes(void)
{
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

	I2C_Cmd(ENABLE);
	I2C_Init(I2C_SPEED, I2C_SLAVE_ADDRESS7, I2C_DUTYCYCLE_2, I2C_ACK_CURR,I2C_ADDMODE_7BIT, 16);
	GPIO_Init(GPIOB, ((GPIO_Pin_TypeDef)(GPIO_PIN_4|GPIO_PIN_5)), GPIO_MODE_OUT_OD_HIZ_FAST);
}

void write8(uint16_t reg, uint8_t data)
{
	while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_W;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;


	I2C_SendData((uint8_t)(reg >> 8));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_SendData((uint8_t)(reg & 0x00FF));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_SendData(data);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_GenerateSTOP(ENABLE);
}

void writeN(uint16_t reg, uint8_t *pData, uint8_t Length)
{
	uint16_t cnt;

	while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_W;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	I2C_SendData((uint8_t)(reg >> 8));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_SendData((uint8_t)(reg & 0x00FF));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	for (cnt = 0; cnt < (Length - 1); cnt++)
	{
		I2C_SendData(*pData);
		while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			;
		pData++;
	}
	I2C_SendData(*pData);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_GenerateSTOP(ENABLE);
}

void write16(uint16_t reg, uint16_t data)
{
	i2cBuf[0] = data >> 8;
	i2cBuf[1] = data & 0x00ff;

	writeN(reg, i2cBuf, 2);
}

void write32(uint16_t reg, uint32_t data)
{
	i2cBuf[0] = (data >> 24) & 0xff;
	i2cBuf[1] = (data >> 16) & 0xff;
	i2cBuf[2] = (data >> 8) & 0xff;
	i2cBuf[3] = data & 0xff;

	writeN(reg, i2cBuf, 4);
}

uint8_t read8(uint16_t reg)
{
	while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_W;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	I2C_SendData((uint8_t)(reg >> 8));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_SendData((uint8_t)(reg & 0x00FF));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_R;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	I2C_AcknowledgeConfig(I2C_ACK_NONE);
	while (I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
		;
	uint8_t data = I2C_ReceiveData(); 
	I2C_GenerateSTOP(ENABLE);

	return data;
}

uint16_t read16(uint16_t reg)
{
	readN(reg, i2cBuf, 2);
	return *(uint16_t *)i2cBuf;
}

uint32_t read32(uint16_t reg)
{
	uint16_t result;
	readN(reg, i2cBuf, 4);
	result = ((uint32_t)i2cBuf[0]<<24) + ((uint32_t)i2cBuf[1]<<16) + ((uint32_t)i2cBuf[2]<<8)  + (uint32_t)i2cBuf[3];
	return result;
}

void readN(uint16_t reg, uint8_t *pData, uint16_t Length)
{
	uint16_t cnt;

	while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_W;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		;

	I2C_SendData((uint8_t)(reg >> 8));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
	I2C_SendData((uint8_t)(reg & 0x00FF));
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;

	I2C_GenerateSTART(ENABLE);
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT))
		;

	SET_R;
	while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		;

	for (cnt = 0; cnt < (Length - 1); cnt++)
	{
		I2C_AcknowledgeConfig(I2C_ACK_CURR);
		while (I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
			;
		*pData = I2C_ReceiveData();
		pData++;
	}
	I2C_AcknowledgeConfig(I2C_ACK_CURR);
	while (I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) == RESET)
		;
	*pData = I2C_ReceiveData();

	I2C_GenerateSTOP(ENABLE);
}

