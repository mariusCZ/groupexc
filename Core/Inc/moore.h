/*
 * moore.h
 *
 *  Created on: 2020-11-30
 *      Author: vartotojas
 */

#ifndef INC_MOORE_H_
#define INC_MOORE_H_

#include "usbh_core.h"
#include "stm32l475e_iot01.h"
#include "usbh_msc.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio_dma.h"
#include <stdint.h>
#include "stm_hsensor.h"
#include "stm_tsensor.h"
#include "stm_psensor.h"
#include "magdriver.h"
#include "lsm6dsl_accel.h"

#define READCNT 10 /* Sensor read count for averaging */
#define BUFFER_SIZE 300 /* Buffer size for storage functions */
#define ACC_SPEED 25 /* In how many ms a reading will be done for accelerometer */

void stateOperations(uint16_t min, uint16_t s, uint8_t uflag);
void stateMachine(uint8_t uflag);

/* Project state machine enum */
typedef enum {
	STATE_IDLE = 0,
	STATE_READING,
	STATE_DATAMANAGE,
	STATE_STORING,
	STATE_ERROR,
}Moore_TypeDef;

extern FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
extern FIL MyFile;                   /* File object */
extern char USBDISKPath[4];          /* USB Host logical drive path */
extern USBH_HandleTypeDef hUSBHost;  /* USB Host handle */
extern Moore_TypeDef MachineState;	 /* To obtain initial machine state */

#endif /* INC_MOORE_H_ */
