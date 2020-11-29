/*
 * lsm6dsl_accel.c
 *
 *  Created on: 2020-11-29
 *      Author: vartotojas
 */

#include "lsm6dsl_accel.h"

uint8_t lsm6dsl_accel_read_id(void)
{
	uint8_t data;

	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//	if (data == 106){printf("\r\n Correct ID Read: %d\r\n", data);}
//	else{printf("\r\n Incorrect ID Read: %d\r\n", data);}
	return data;
}

void lsm6dsl_accel_init (void)
{
	uint8_t data_CTRL1_XL;
	uint8_t data_CTRL3_C;
	//uint8_t check_data_CTRL1_XL;
	//uint8_t check_data_CTRL3_C;

	/*Read CTRL1_XL*/
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &data_CTRL1_XL, 1, HAL_MAX_DELAY);

	/* Write value to CTRL1_XL register: FS and Data Rate */
	data_CTRL1_XL &= 0x00;
	data_CTRL1_XL |= LSM6DSL_ODR_104Hz;
	data_CTRL1_XL |= LSM6DSL_ACC_FULLSCALE_2G;
	HAL_I2C_Mem_Write(&hI2cHandler, lsm6dsl_W, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &data_CTRL1_XL, 1, HAL_MAX_DELAY);

	/*Read CTRL3_C*/
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL3_C, I2C_MEMADD_SIZE_8BIT, &data_CTRL3_C, 1, HAL_MAX_DELAY);

	/*Write value to CTRL3_C register: BDU and Auto-increment*/
	data_CTRL3_C |= LSM6DSL_BDU_CONTINUOS;
	data_CTRL3_C &= LSM6DSL_ACC_IF_INC_DISABLED;
	HAL_I2C_Mem_Write(&hI2cHandler, lsm6dsl_W, LSM6DSL_ACC_CTRL3_C, I2C_MEMADD_SIZE_8BIT, &data_CTRL3_C, 1, HAL_MAX_DELAY);

	/*check contents of CTRL1_XL and CTRL3_C*/
	//HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &check_data_CTRL1_XL, 1, 10000);
	//HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL3_C, I2C_MEMADD_SIZE_8BIT, &check_data_CTRL3_C, 1, 10000);
	//printf("\r\n(CTRL1_XL: %d, CTRL3_C: %d)\r\n", check_data_CTRL1_XL, check_data_CTRL3_C);
}

void lsm6dsl_accel_deinit(void)
{
	uint8_t ctrl = 0x00;

	/* Read control register 1 value */
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);

	/* Clear ODR bits */
	ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

	/* Set Power down */
	ctrl |= LSM6DSL_ODR_POWER_DOWN;

	/* write back control register */
	HAL_I2C_Mem_Write(&hI2cHandler, lsm6dsl_W, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);
}

void lsm6dsl_accel_lowpower (void)
{
	uint8_t ctrl = 0x00;

	/* Read CTRL6_C value */
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL6_C, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);
	//ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C);

	/* Clear Low Power Mode bit */
	ctrl |= ~(0x10);

	/* Set Low Power Mode */
	ctrl |= LSM6DSL_ACC_LP_XL_ENABLED;

	/* write back control register */
	HAL_I2C_Mem_Write(&hI2cHandler, lsm6dsl_W, LSM6DSL_ACC_CTRL6_C, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);
	//SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C, ctrl);
}

void lsm6dsl_accel_readxyz (int16_t *pdata)
{
	uint8_t X_LSb, X_MSb, Y_LSb, Y_MSb, Z_LSb, Z_MSb;
	int16_t RawData_X, RawData_Y, RawData_Z;
	uint8_t ctrl = 0;
	float sensitivity = 0;

	/*read contents of CTRL1_XL register*/
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, HAL_MAX_DELAY);

	/*Read output registers*/
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, &X_LSb, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTX_H_XL, I2C_MEMADD_SIZE_8BIT, &X_MSb, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTY_L_XL, I2C_MEMADD_SIZE_8BIT, &Y_LSb, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTY_H_XL, I2C_MEMADD_SIZE_8BIT, &Y_MSb, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTZ_L_XL, I2C_MEMADD_SIZE_8BIT, &Z_LSb, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_OUTZ_H_XL, I2C_MEMADD_SIZE_8BIT, &Z_MSb, 1, HAL_MAX_DELAY);

	/*combine MSb and LSb data of each plane*/
	RawData_X = ((uint16_t)X_MSb << 8) + ((uint16_t)X_LSb);
	RawData_Y = ((uint16_t)Y_MSb << 8) + ((uint16_t)Y_LSb);
	RawData_Z = ((uint16_t)Z_MSb << 8) + ((uint16_t)Z_LSb);

	switch(ctrl &= 0x00)
	{
		case LSM6DSL_ACC_FULLSCALE_2G:sensitivity = LSM6DSL_ACC_SENSITIVITY_2G;
			break;
		case LSM6DSL_ACC_FULLSCALE_4G:sensitivity = LSM6DSL_ACC_SENSITIVITY_4G;
			break;
		case LSM6DSL_ACC_FULLSCALE_8G:sensitivity = LSM6DSL_ACC_SENSITIVITY_8G;
	    	break;
	    case LSM6DSL_ACC_FULLSCALE_16G:sensitivity = LSM6DSL_ACC_SENSITIVITY_16G;
	    	break;
	}

	/* Obtain the mg value for the three axis */
	pdata[0] = (int16_t)(RawData_X * sensitivity);
	pdata[1] = (int16_t)(RawData_Y * sensitivity);
	pdata[2] = (int16_t)(RawData_Z * sensitivity);

}

uint8_t lsm6dsl_accel_datacheck (void)
{
	uint8_t status_reg_read = 0;

	/*Read Status Reg and print it*/
	HAL_I2C_Mem_Read(&hI2cHandler, lsm6dsl_R, LSM6DSL_ACC_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_reg_read, 1, HAL_MAX_DELAY);

	/*Check contents of Status Reg, specifically XLDA bit*/
	return Check_Bit(status_reg_read,0);
}
