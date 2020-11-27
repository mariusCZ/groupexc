/*
 * i2cdriver.h
 *
 *  Created on: 2020-10-06
 *      Author: vartotojas
 */

#ifndef SRC_I2CDRIVER_H_
#define SRC_I2CDRIVER_H_

#include "stm32l4xx_hal.h"

#ifndef DISCOVERY_I2Cx_TIMING
#define DISCOVERY_I2Cx_TIMING                     ((uint32_t)0x00702681)
#endif /* DISCOVERY_I2Cx_TIMING */

I2C_HandleTypeDef hi2c2;

/* Prototypes */

void     MAGNET_Write(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t  MAGNET_Read(uint8_t addr, uint8_t reg);
uint16_t MAGNET_ReadMultiple(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* SRC_I2CDRIVER_H_ */
