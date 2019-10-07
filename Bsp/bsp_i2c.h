#ifndef _I2C_EEPROM_H
#define _I2C_EEPROM_H

#include "stm8s.h"

#define I2C_SLAVE_ADDRESS7        0xA0
#define I2C_SPEED                 100000


void I2C_Initializes(void);
void readN(uint16_t reg, uint8_t *pData, uint16_t Length);
uint8_t read8(uint16_t reg);
uint16_t read16(uint16_t reg);
uint32_t read32(uint16_t reg);
void write32(uint16_t reg, uint32_t data);
void write16(uint16_t reg, uint16_t data);
void writeN(uint16_t reg, uint8_t *pData, uint8_t Length);
void write8(uint16_t reg, uint8_t data);

#endif
