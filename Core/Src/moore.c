#include "moore.h"

static void Error_Handler(void);

void stateMachine(uint8_t uflag) {
	static uint16_t s = 0, min = 0, lastuw = 0;
	/* The state machine's input */
	static uint8_t input = 0;
	/* File name where data will be stored */
	const char filename1[] = "datalog.txt", filename2[] = "accel.txt";

	/* If running with USB OTG, mount file system and create file */
	if (uflag) {
		/* Register the file system object to the FatFs module */
		if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
		{
			/* FatFs Initialization Error */
			Error_Handler();
		}
		else
		{
			/* Create and Open a new text file object with write access */
			if(f_open(&MyFile, filename1, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
			  /* file Open for write Error */
			  Error_Handler();
			}
			/* Close file */
			f_close(&MyFile);
			/* Create and Open a new text file object with write access */
			if(f_open(&MyFile, filename2, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
			  /* file Open for write Error */
			  Error_Handler();
			}
			/* Close file */
			f_close(&MyFile);
		}
	}

	uwTick = 0;
	while(1) {
		/* If User button pressed, quit state machine */
		if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) break;
		/* Check if uwTick, which is a variable that is incremented each millisecond
		 * by the system clock, is equal to a second, if so, set the input high.
		 */
		if (uwTick >= 1000) {
			input = 1;
			s++;
			uwTick = 0;
			/* Keep track of seconds and minutes passed */
			if (s >= 60) {
				s = 0;
				min++;
			}
		}
		/* State machine's switch statement */
		switch(MachineState) {
		case STATE_IDLE:
			if (input) {
				MachineState = STATE_READING;
			}
			else {
				if((uwTick - lastuw) >= ACC_SPEED) {
					lastuw = uwTick;
					if(uflag) {
						if(f_open(&MyFile, filename2, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
						{
							/* file Open for write Error */
							Error_Handler();
						}
					}
					stateOperations(min,s,uflag);
				}
			}
			break;

		case STATE_READING:
			printf("Reading state\n");
			input = 0;
			/* If working with USB OTG, open the file for appending */
			if(uflag) {
				if(f_open(&MyFile, filename1, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
				{
					/* file Open for write Error */
					Error_Handler();
				}
			}
			if (input) MachineState = STATE_IDLE;
			else {
				stateOperations(0,0,uflag);
				MachineState = STATE_DATAMANAGE;
			}
			break;

		case STATE_DATAMANAGE:
			printf("Data state\n");
			if (input) MachineState = STATE_ERROR;
			else {
				/* Pass the time variables to state operations */
				stateOperations(min,s,uflag);
				MachineState = STATE_STORING;
			}
			break;

		case STATE_STORING:
			printf("Storing state\n");
			stateOperations(0,0,uflag);
			if (input) MachineState = STATE_READING;
			else MachineState = STATE_IDLE;
			break;

		case STATE_ERROR:
			printf("Error state\n");
			stateOperations(0,0,uflag);
			if (!input) MachineState = STATE_STORING;
			break;

		default:
			break;
		}
	}
	printf("Done writing\n");
	/* Unlink the USB disk I/O driver once state machine is done */
	FATFS_UnLinkDriver(USBDISKPath);
}


void stateOperations(uint16_t min, uint16_t s, uint8_t uflag) {
	/* This function is a supplement of the state machine, since this is where
	 * all the operations that are done in the different states are stored.
	*/
	/* Variables for general state machine operation */
	static float sensors[3];
	static uint8_t text[][30] = {"Temperature: ", "Humidity: ", "Pressure: "};
	static int16_t magsensbuf[READCNT][3], magsens[3];
	uint32_t byteswritten;
	FRESULT res;
	static char buf[BUFFER_SIZE] = {0};
	/* Variables for accelerometer, including its moving average */
	static uint8_t readIndex = 0;
	static int16_t readings[READCNT][3] = {{0}}, average[3] = {0}, total[3] = {0};

	/* State machine switch */
	switch (MachineState) {
	case STATE_IDLE:
		/* MOVING AVERAGE SECTION */
		/* This section applies a moving average to the accelerometer's
		 * data, since it can be quite noisy due to detecting any type of
		 * vibration.
		 */
		for (int i = 0; i < 3; i++) {
			total[i] = total[i] - readings[readIndex][i];
			if(i == 2) {
				if(lsm6dsl_accel_datacheck())
					lsm6dsl_accel_readxyz(readings[readIndex]);
			}
		}
		for (int i = 0; i < 3; i++) {
			total[i] = total[i] + readings[readIndex][i];
			average[i] = total[i] / READCNT;
		}
		readIndex++;
		if (readIndex >= READCNT) readIndex = 0;
		/* MOVING AVERAGE SECTION END */
		/* ACCELEROMETER STORE SECTION */
		char accbuf[BUFFER_SIZE] = {0};
		char textmag[][5] = {"X:  \0", "Y:  \0", "Z:  \0"}, tmag[] = {"Accelerometer: "}, bufTime[32] = {0};
		sprintf(bufTime,"%d:%d:%ld ", min, s, uwTick);
		strcat(accbuf, bufTime);
		strcat(accbuf, tmag);
		for (int i = 0; i < 3; i++) {
			char numBuf[16] = {0};
			sprintf(numBuf, "%d ", average[i]);
			strcat(accbuf, textmag[i]);
			strcat(accbuf, numBuf);
		}
		printf("%s\n", accbuf);

		if (uflag) {
			res = f_write(&MyFile, accbuf, strlen(accbuf), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				/* file Write or EOF Error */
				Error_Handler();
			}
			res = f_write(&MyFile, "\r\n", 2, (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				/* file Write or EOF Error */
				Error_Handler();
			}
			f_close(&MyFile);
		}
		/* ACCELEROMETER STORE SECTION END */
		break;
	case STATE_READING:
		/* Read the three workspace sensors */
		sensors[0] = BSP_TSENSOR_ReadTemp();
		sensors[1] = BSP_HSENSOR_ReadHumidity();
		sensors[2] = BSP_PSENSOR_ReadPressure();
		uint8_t magCnt = 0;
		/* Read set amount of magnetometer sensor */
		while (magCnt <= READCNT) {
			if (MAGNET_DataCheck()) {
				  /* Read the data */
				  MAGNET_ReadXYZ(magsensbuf[magCnt]);
				  magCnt++;
			}
		}
		break;
	case STATE_DATAMANAGE:
	{
		/* AVERAGE SECTION
		 * A simple arithmetic mean is done to
		 * reduce noise in the magnetometer sensor */
		for (int i = 0; i < 3; i++) {
			int16_t average = 0;
			for (int j = 0; j < READCNT; j++) {
				average += magsensbuf[j][i];
			}
			average = average / READCNT;
			magsens[i] = average;
		}
		/* AVERAGE SECTION END */

		/* SINGLE READ SENSOR SECTION
		 * This section creates char array that stores all the names
		 * and the values of the sensors for writing to the USB flash drive. */
		uint8_t count = 0, lastcnt = 0;
		for (int i = 0; i < 3; i++) {
			/* Buffer arrays */
			char bufTime[32] = {0}, bufFloat[16] = {0}, buftext[100] = {0};
			/* Convert float to string */
			sprintf(bufFloat, "%f ", sensors[i]);
			/* Store time value once */
			if(!i) {
				sprintf(bufTime,"%d:%d:%ld ", min, s, uwTick);
				strcat(buftext, bufTime);
			}
			/* Store sensor names and values within main buffer */
			strcat(buftext, (char*)text[i]);
			strcat(buftext, bufFloat);

			/* To prevent large spaces between the different sensor
			 * values, the non zero characters are counted and manually placed
			 * within the printing buffer without using strcat.
			 */
			for (int j = 0; j < sizeof(buftext); j++) {
				if (buftext[j] != '\0') count++;
			}

			for (int j = lastcnt; j < count; j++) buf[j] = buftext[j-lastcnt];
			lastcnt = count;
		}
		/* SINGLE READ SENSOR SECTION END */

		/* MAGNET READ SENSOR SECTION
		 * Magnetometer data stored within character array, as before. */
		char textmag[][5] = {"X:  \0", "Y:  \0", "Z:  \0"}, tmag[] = {"Magnetometer: "};
		strcat(buf, tmag);
		for (int i = 0; i < 3; i++) {
			char magBuf[16] = {0};
			sprintf(magBuf, "%d ", magsens[i]);
			strcat(buf, textmag[i]);
			strcat(buf, magBuf);
		}
		/* MAGNET READ SENSOR SECTION */
		break;
	}
	case STATE_STORING:
		if (uflag) {
			res = f_write(&MyFile, buf, strlen(buf), (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				/* file Write or EOF Error */
				Error_Handler();
			}
			res = f_write(&MyFile, "\r\n", 2, (void *)&byteswritten);
			if((byteswritten == 0) || (res != FR_OK))
			{
				/* file Write or EOF Error */
				Error_Handler();
			}
			/* Close the file and open it again once the appropriate state is reached.
			 * The reason for this is to ensure that the data that was written is properly
			 * stored within the file in the case of an unexpected system shutdown.
			 */
			f_close(&MyFile);
		}
		printf("%s\n", buf);
		for (int i = 0; i < BUFFER_SIZE; i++) buf[i] = 0;
		break;
	case STATE_ERROR:
		printf("Something bad happened\n");
		break;
	default:
		break;
	}
}

static void Error_Handler(void)
{
  printf("Error\r\n");
  while(1)
  {
  }
}
