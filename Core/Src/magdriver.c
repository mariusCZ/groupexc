/*
 * magdriver.c
 *
 *  Created on: 2020-10-06
 *      Author: vartotojas
 */

#include "magdriver.h"
#include "main.h"

void MAGNET_Init(struct Regs regs) {
	/* Write all the register values */
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG1, regs.reg1);
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2, regs.reg2);
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, regs.reg3);
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG4, regs.reg4);
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG5, regs.reg5);
}

void MAGNET_DeInit(void) {
	uint8_t ctrl = 0x00;

	/* Read the control register 3 */
	ctrl = MAGNET_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);
	/* Disable selection mode */
	ctrl &= ~(LIS3MDL_MAG_SELECTION_MODE);
	/* Enable powerdown2 mode */
	ctrl |= LIS3MDL_MAG_POWERDOWN2_MODE;
	/* Write back to control register 3 */
	MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);
}

uint8_t MAGNET_ReadID(void)
{
  /* Read value at Who am I register address */
  return (MAGNET_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_WHO_AM_I_REG));
}

void MAGNET_MagLowPower(uint16_t status)
{
  uint8_t ctrl = 0;

  /* Read control register 1 value */
  ctrl = MAGNET_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x20);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LIS3MDL_MAG_CONFIG_LOWPOWER_MODE;
  }else
  {
    ctrl |= LIS3MDL_MAG_CONFIG_NORMAL_MODE;
  }

  /* write back control register */
  MAGNET_Write(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG3, ctrl);
}

uint8_t MAGNET_DataCheck(void) {
	uint8_t ctrl = 0;

	/* Read the status register */
	ctrl = MAGNET_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_STATUS_REG);
	/* Check if fifth bit is 1, if so return 1 */
	if (CHECK_BIT(ctrl, 4)) return 1;
	else return 0;
}

void MAGNET_ReadXYZ(int16_t* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlm= 0;
  uint8_t buffer[6];
  uint8_t i = 0;
  float sensitivity = 0;

  /* Read the magnetometer control register content */
  ctrlm = MAGNET_Read(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_CTRL_REG2);

  /* Read output register X, Y & Z acceleration */
  MAGNET_ReadMultiple(LIS3MDL_MAG_I2C_ADDRESS_HIGH, LIS3MDL_MAG_OUTX_L, buffer, 6);

  for(i=0; i<3; i++)
  {
    pnRawData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }

  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL_REG2 */
  switch(ctrlm & 0x60)
  {
  case LIS3MDL_MAG_FS_4_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_4GA;
    break;
  case LIS3MDL_MAG_FS_8_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_8GA;
    break;
  case LIS3MDL_MAG_FS_12_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_12GA;
    break;
  case LIS3MDL_MAG_FS_16_GA:
    sensitivity = LIS3MDL_MAG_SENSITIVITY_FOR_FS_16GA;
    break;
  }

  /* Obtain the mGauss value for the three axis */
  for(i=0; i<3; i++)
  {
    pData[i]=( int16_t )(pnRawData[i] * sensitivity);
  }
}
