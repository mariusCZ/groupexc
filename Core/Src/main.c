/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   Main program body
  *          This sample code shows how to use FatFs with USB disk drive.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license SLA0044,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* TODO: Remove magic numbers
 * TODO: Add comments
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm_hsensor.h"
#include "stm_tsensor.h"
#include "stm_psensor.h"
#include "magdriver.h"
#include "lsm6dsl_accel.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define READCNT 10
#define BUFFER_SIZE 300
#define ACC_SPEED 25 /* In how many ms a reading will be done for accelerometer */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSBHost; /* USB Host handle */

MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
Moore_TypeDef MachineState = STATE_IDLE;
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

static void SystemClock_Config(void);
static void Error_Handler(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  /* Enable Power Clock*/
  __HAL_RCC_PWR_CLK_ENABLE();

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (8 data bit + 0 parity bit) :
        BE CAREFUL :
             Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = None parity
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USART1;
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Initialize register values */
  struct Regs regs;
  regs.reg1 = LIS3MDL_MAG_TEMPSENSOR_DISABLE | LIS3MDL_MAG_OM_XY_HIGH | LIS3MDL_MAG_ODR_80_HZ;
  regs.reg2 = LIS3MDL_MAG_FS_4_GA | LIS3MDL_MAG_REBOOT_DEFAULT | LIS3MDL_MAG_SOFT_RESET_DEFAULT;
  regs.reg3 = LIS3MDL_MAG_CONFIG_NORMAL_MODE | LIS3MDL_MAG_CONTINUOUS_MODE;
  regs.reg4 = LIS3MDL_MAG_OM_Z_HIGH | LIS3MDL_MAG_BLE_LSB;
  regs.reg5 = LIS3MDL_MAG_BDU_MSBLSB;

  /* Initialize magnetometer */
  MAGNET_Init(regs);

  /* Read the magnetometer ID */
  uint8_t mID = MAGNET_ReadID();
  /* Check whether the ID is correct */
  if (mID == 61) printf("ID correct, sensor communications operational.\n");
  else {
	printf("ID incorrect, sensor communication non-functional.\n");
	while (1) {}
  }

  /* Initialize Accelerometer */
  lsm6dsl_accel_init();
  /* Check whether the ID is correct */
  if(lsm6dsl_accel_read_id() == 106) printf("Accelerometer ID correct \n");
  else {
	  printf("Accelerometer ID incorrect, sensor communication non-functional.\n");
	  while (1) {}
  }

  /* Intialize the workspace sensors */
  BSP_HSENSOR_Init();
  BSP_TSENSOR_Init();
  BSP_PSENSOR_Init();

  /* Check if program used with or without USB OTG */
  /* This is only available if ENABLE_WITHOUT_USB is set to 1 in main.h */
  uint8_t c = 0;
  if (ENABLE_WITHOUT_USB) {
	  printf("Press y if you would like to run without USB\n");
	  HAL_UART_Receive(&UartHandle, (uint8_t *)&c, 1, 0xFFFF);
	  if (c == 'y' || c == 'Y')
		  /* Run state machine without USB OTG */
		  while(1) {stateMachine(0);}
  }

  /* Initialize the USB and run the USB state machine */
  if(USBinit() == 0)
  {
    while (1)
    {
      /* USB Host Background task */
      USBH_Process(&hUSBHost);

      /* Mass Storage Application State Machine */
      switch(Appli_state)
      {
      case APPLICATION_START:
    	/* Start state machine with USB OTG */
        stateMachine(1);
        Appli_state = APPLICATION_IDLE;
        break;

      case APPLICATION_IDLE:
      default:
        break;
      }
    }
  }

  /* Infinite loop */
  for(;;);
}

uint8_t USBinit() {
	/*##-1- Link the USB Host disk I/O driver ##################################*/
	uint8_t ret = FATFS_LinkDriver(&USBH_Driver, USBDISKPath);

	/*##-2- Init Host Library ################################################*/
	USBH_Init(&hUSBHost, USBH_UserProcess, 0);

	/*##-3- Add Supported Class ##############################################*/
	USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);

	/*##-4- Start Host Process ###############################################*/
	USBH_Start(&hUSBHost);

	return ret;
}

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

/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
    break;

  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_IDLE;
    f_mount(NULL, (TCHAR const*)"", 0);
    FATFS_UnLinkDriver(USBDISKPath);
    break;

  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_START;
    FATFS_LinkDriver(&USBH_Driver, USBDISKPath);
    f_mount(&USBH_fatfs, "", 0);

    break;

  default:
    break;
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 1
  *            PLL_N                          = 20
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Configure LSE Drive Capability */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /* Initialize the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                                    |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize the CPU, AHB and APB busses clocks  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize the USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable MSI Auto calibration */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  printf("Error\r\n");
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
