/*
 * i2cdriver.c
 *
 *  Created on: 2020-10-06
 *      Author: vartotojas
 */

#include "i2cdriver.h"
#include "stm32l475e_iot01.h"

/* Static prototypes */

static HAL_StatusTypeDef I2C2_Read(I2C_HandleTypeDef *i2c_handler, uint8_t addr, uint16_t reg, uint16_t addrSize, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef I2C2_Write(I2C_HandleTypeDef *i2c_handler, uint8_t addr, uint16_t reg, uint16_t addrSize, uint8_t *buf, uint16_t len);

static HAL_StatusTypeDef I2C2_Read(I2C_HandleTypeDef *i2c_handler, uint8_t addr, uint16_t reg, uint16_t addrSize, uint8_t *buf, uint16_t len) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read(i2c_handler, addr, (uint16_t)reg, addrSize, buf, len, 1000);

	return status;
}

static HAL_StatusTypeDef I2C2_Write(I2C_HandleTypeDef *i2c_handler, uint8_t addr, uint16_t reg, uint16_t addrSize, uint8_t *buf, uint16_t len) {
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(i2c_handler, addr, (uint16_t)reg, addrSize, buf, len, 1000);

	return status;
}

void MAGNET_Write(uint8_t addr, uint8_t reg, uint8_t val) {
	I2C2_Write(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&val, 1);
}

uint8_t MAGNET_Read(uint8_t addr, uint8_t reg) {
	uint8_t read_val = 0;
	I2C2_Read(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_val, 1);
	return read_val;
}

uint16_t MAGNET_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
	return I2C2_Read(&hI2cHandler, addr, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, len);
}
