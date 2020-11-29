/*
 * lsm6dsl_accel.h
 *
 *  Created on: Nov 15, 2020
 *      Author: Connah Bailey
 */

#ifndef INC_LSM6DSL_ACCEL_H_
#define INC_LSM6DSL_ACCEL_H_

#include "stm32l4xx_hal.h"
#include "main.h"

/************** Device Address *****************/
#define lsm6dsl_R		(uint8_t)0xD5 /**I2C Read**/
#define lsm6dsl_W		(uint8_t)0xD4 /**I2C Write**/

/************** WHO AM I *******************/
#define LSM6DSL_ACC_WHO_AM_I		(uint8_t)0x0f

/************** Device Register *******************/
#define LSM6DSL_ACC_CTRL1_XL		(uint8_t)0x10
#define LSM6DSL_ACC_CTRL3_C			(uint8_t)0x12
#define LSM6DSL_ACC_CTRL6_C			(uint8_t)0x15
#define LSM6DSL_ACC_STATUS_REG		(uint8_t)0x1E
#define LSM6DSL_ACC_OUTX_L_XL		(uint8_t)0x28
#define LSM6DSL_ACC_OUTX_H_XL		(uint8_t)0x29
#define LSM6DSL_ACC_OUTY_L_XL		(uint8_t)0x2A
#define LSM6DSL_ACC_OUTY_H_XL		(uint8_t)0x2B
#define LSM6DSL_ACC_OUTZ_L_XL		(uint8_t)0x2C
#define LSM6DSL_ACC_OUTZ_H_XL		(uint8_t)0x2D

/************** Accelerometer Full Scale Selection *******************/
#define LSM6DSL_ACC_FULLSCALE_2G		((uint8_t)0x00) /*plus/minus 2g*/
#define LSM6DSL_ACC_FULLSCALE_4G		((uint8_t)0x08) /*plus/minus 4g*/
#define LSM6DSL_ACC_FULLSCALE_8G		((uint8_t)0x0C) /*plus/minus 8g*/
#define LSM6DSL_ACC_FULLSCALE_16G		((uint8_t)0x04) /*plus/minus 16g*/

/************** Accelerometer Full Scale Sensitivity *******************/
#define LSM6DSL_ACC_SENSITIVITY_2G     ((float)0.061f)  /*!< accelerometer sensitivity with 2 g full scale  [mgauss/LSB]*/
#define LSM6DSL_ACC_SENSITIVITY_4G     ((float)0.122f)  /*!< accelerometer sensitivity with 4 g full scale  [mgauss/LSB]*/
#define LSM6DSL_ACC_SENSITIVITY_8G     ((float)0.244f)  /*!< accelerometer sensitivity with 8 g full scale  [mgauss/LSB]*/
#define LSM6DSL_ACC_SENSITIVITY_16G    ((float)0.488f)  /*!< accelerometer sensitivity with 12 g full scale [mgauss/LSB]*/

/************** Accelerometer Power Mode selection *******************/
#define LSM6DSL_ACC_LP_XL_DISABLED     ((uint8_t)0x00) /**LP disabled**/
#define LSM6DSL_ACC_LP_XL_ENABLED      ((uint8_t)0x10) /**LP enabled**/

/************** Output Data Rate *******************/
#define LSM6DSL_ODR_BITPOSITION      ((uint8_t)0xF0) /**Output Data Rate bit position**/
#define LSM6DSL_ODR_POWER_DOWN       ((uint8_t)0x00) /**Power Down mode      **/
#define LSM6DSL_ODR_2Hz				 ((uint8_t)0xB0) /**Low Power mode only  **/
#define LSM6DSL_ODR_13Hz             ((uint8_t)0x10) /**Low Power mode       **/
#define LSM6DSL_ODR_26Hz             ((uint8_t)0x20) /**Low Power mode       **/
#define LSM6DSL_ODR_52Hz             ((uint8_t)0x30) /**Low Power mode       **/
#define LSM6DSL_ODR_104Hz            ((uint8_t)0x40) /**Normal mode          **/
#define LSM6DSL_ODR_208Hz            ((uint8_t)0x50) /**Normal mode          **/
#define LSM6DSL_ODR_416Hz            ((uint8_t)0x60) /**High Performance mode**/
#define LSM6DSL_ODR_833Hz            ((uint8_t)0x70) /**High Performance mode**/
#define LSM6DSL_ODR_1660Hz           ((uint8_t)0x80) /**High Performance mode**/
#define LSM6DSL_ODR_3330Hz           ((uint8_t)0x90) /**High Performance mode**/
#define LSM6DSL_ODR_6660Hz           ((uint8_t)0xA0) /**High Performance mode**/

/************** Check an Individual bit *******************/
#define Check_Bit(var,pos) ((var) & (1<<(pos)))

/************** Block Data Update *************************/
#define LSM6DSL_BDU_CONTINUOS               ((uint8_t)0x00)
#define LSM6DSL_BDU_BLOCK_UPDATE            ((uint8_t)0x40)

/************** Auto-increment ****************************/
#define LSM6DSL_ACC_IF_INC_DISABLED    ((uint8_t)0x00)
#define LSM6DSL_ACC_IF_INC_ENABLED     ((uint8_t)0x04)

uint8_t lsm6dsl_accel_read_id(void);
void lsm6dsl_accel_init (void);
void lsm6dsl_accel_deinit (void);
void lsm6dsl_accel_readxyz (int16_t *pdata);
uint8_t lsm6dsl_accel_datacheck (void);
void lsm6dsl_accel_lowpower (void);

#endif /* INC_LSM6DSL_ACCEL_H_ */
