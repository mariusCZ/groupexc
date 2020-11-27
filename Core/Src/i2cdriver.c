/*
 * i2cdriver.c
 *
 *  Created on: 2020-10-06
 *      Author: vartotojas
 */

#include "i2cdriver.h"
#include "stm32l475e_iot01.h"

/* Static prototypes */

void MAGNET_Write(uint8_t addr, uint8_t reg, uint8_t val) {
	I2Cx_WriteMultiple(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&val, 1);
}

uint8_t MAGNET_Read(uint8_t addr, uint8_t reg) {
	uint8_t read_val = 0;
	I2Cx_ReadMultiple(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_val, 1);
	return read_val;
}

uint16_t MAGNET_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
	return I2Cx_ReadMultiple(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}
