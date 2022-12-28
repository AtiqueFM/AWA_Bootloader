/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "FLash.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CONFIG_DATA_FUNC __attribute__((section(".mysection")))
#define SHOW_UART2_STATUS
//#define DISABLE
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/*Local variable*/
bootloader_handle_t global_BOOTLOADER_HANLDE_STRUCT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

Buffer_t UART_Buffer;
Buffer_t FLash_UART_read_buffer;

uint16_t UART_Rx_Len = (uint16_t)UART2_RX_LENGTH_IN_BYTES;
uint8_t tx_buffer[100];
uint8_t re_init_rx;
uint8_t msg_buffer[100];
uint8_t file_info_flag = 0;//if 1 -> no of rows, if 2 -> no of bytes in the hex file
uint32_t no_of_rows = 0;
uint16_t live_data_row = 0;
bootloader_status_hanlde_t BOOTLOADER_STATUSBITS;
uint32_t lu16_cal_crc_16_bits = 0;
bootloader_handle_t BOOTLOADER_HANLDE_STRUCT;

extern uint8_t data_bytes;
extern uint32_t FLASHStartingAddress;
extern uint16_t recv_packet_counter;

typedef void (*pFunction)(void);

#if 0
void go2App(void);
#else
void go2App(uint32_t address);
#endif

void configuration_data(void);

static void load_default_confif_data_in_flash(void);
static void erase_program_partition_sectors(uint8_t sector);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 0
void FOTA_TimerInit(void)
{
	/*
	 * Hear beat timer at 1000 ms
	 */
	HAL_TIM_Base_Start_IT(&htim3);
}
void FOTA_Hearbeat(void)
{
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}
#endif

#if 0
void go2App(void)
{
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	if(((*(uint32_t*)HEXFILE_FLASHADDRESS) & 0x2ffe0000) == 0x20020000)
	{
		//App start
		JumpAddress = *(uint32_t*) (HEXFILE_FLASHADDRESS + 4);//Jumping address to the reset handler
		Jump_To_Application = (pFunction)JumpAddress;//Function pointer to reset handler
		//Data received from UART
		putMessages((uint8_t*)"Jumping to application\n\r");
		//Initialize the application stack pointer
//		__set_MSP(*(uint32_t*)HEXFILE_FLASHADDRESS);//set the Main Stack Pointer to the start of the Application Flash memory location
#ifdef DISABLE
		__HAL_RCC_GPIOC_CLK_DISABLE();
		  __HAL_RCC_GPIOD_CLK_DISABLE();
		  __HAL_RCC_GPIOB_CLK_DISABLE();
		  __HAL_RCC_GPIOA_CLK_DISABLE();
		HAL_RCC_DeInit();
		HAL_DeInit();
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;
#endif
		__set_MSP(*(uint32_t*)HEXFILE_FLASHADDRESS);//set the Main Stack Pointer to the start of the Application Flash memory location
		//jump to the application
		Jump_To_Application();//Execute the Application program
    //printf('C');

	}
	else{
		//Application code not present
	}
}
#else
void go2App(uint32_t address)
{
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	if(((*(uint32_t*)address) & 0x2ffe0000) == 0x20020000)
	{
		//App start
		JumpAddress = *(uint32_t*) (address + 4);//Jumping address to the reset handler
		Jump_To_Application = (pFunction)JumpAddress;//Function pointer to reset handler
		//Data received from UART
		putMessages((uint8_t*)"Jumping to application\n\r");
		//Initialize the application stack pointer
//		__set_MSP(*(uint32_t*)HEXFILE_FLASHADDRESS);//set the Main Stack Pointer to the start of the Application Flash memory location
#ifdef DISABLE
		__HAL_RCC_GPIOC_CLK_DISABLE();
		  __HAL_RCC_GPIOD_CLK_DISABLE();
		  __HAL_RCC_GPIOB_CLK_DISABLE();
		  __HAL_RCC_GPIOA_CLK_DISABLE();
		HAL_RCC_DeInit();
		HAL_DeInit();
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;
#endif
		__set_MSP(*(uint32_t*)address);//set the Main Stack Pointer to the start of the Application Flash memory location
		//jump to the application
		Jump_To_Application();//Execute the Application program
    //printf('C');

	}
	else{
		//Application code not present
	}
}
#endif
void test_memry_clean(void)
{
	for(int i = 4;i<9;i++)
		erase_program_partition_sectors(i);
}

static void set_default_config_data(void)
{
    uint32_t lu32_crc = 0;
    bootloader_handle_t struct_BOOTLODERDATA;
    memset(struct_BOOTLODERDATA.u8array,0,sizeof(struct_BOOTLODERDATA.u8array));
    //strcpy(struct_BOOTLODERDATA.reboot_string,STARTUP_STRING);
    struct_BOOTLODERDATA.BOOTLOADER_CONFIG_DATA.BOOT_SEQUENCE = 1;
    struct_BOOTLODERDATA.BOOTLOADER_CONFIG_DATA.BOOTLOADER_PARTITION_SIZE = 64;
    struct_BOOTLODERDATA.BOOTLOADER_CONFIG_DATA.PARTION_A_FLASH_SIZE = 512;
    struct_BOOTLODERDATA.BOOTLOADER_CONFIG_DATA.PARTION_B_FLASH_SIZE = 384;

    char *pData = NULL;
    pData = (char*)struct_BOOTLODERDATA.u8array;
    uint32_t len = sizeof(struct_BOOTLODERDATA.u8array);
    len -= 4;
    lu32_crc = crc_calc(pData,len);
    struct_BOOTLODERDATA.crc_16_bits = lu32_crc;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //Start the boot-loader state machine
  //FOTA_BoorloaderStateMachine();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  //FOTA_TimerInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Start data on UART interrupt
  putMessages((uint8_t*)"=============================Booting============================\n\r");
  sprintf((char*)msg_buffer,"Firmware version : %d.%d.%d\n\rModification date : %d/%d/%d\n\r",MAJOR_VER,MINOR_VER,BUG_FIX,DD,MM,YY);
  putMessages((uint8_t*)msg_buffer);
  putMessages((uint8_t*)"Bootloader initialized\n\r");
  putMessages((uint8_t*)"Erasing flash...\n\r");
  EraseApplicationMemory();
  putMessages((uint8_t*)"Flash erased!!!\n\r");
#if 0
  HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, (uint16_t)5);
#else
  HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, (uint16_t)4);
#endif
  putMessages((uint8_t*)"BLE UART initiated!!\n\r");
  putMessages((uint8_t*)"Started UART Rx interrupt\n\r");

  while (1)
  {
#if 0
	  if(UART_Rx_Complete)
	  {
		  UART_Rx_Complete = RESET;
		  //Store the data into flash
		  putMessages((uint8_t*)"Writing the HEX file into FLASH\n\r");
		  WriteHEXFILEtoFLash();
		  //Read the Data from FLASH
		  putMessages((uint8_t*)"Reading the HEX file from FLASH\n\r");
		  ReadHEXFILEfromFlash();
		  //Calculate the CRC
		  putMessages((uint8_t*)"Calculating CRC\r\n");
		  uint16_t cal_crc = crc_calc((char*)FLash_UART_read_buffer.UART_Rx_buffer,UART2_RX_LENGTH_IN_BYTES - 2);
		  uint16_t stored_crc = (FLash_UART_read_buffer.UART_Rx_buffer[CRC_Byte_start] << 8);
		  stored_crc |= (FLash_UART_read_buffer.UART_Rx_buffer[CRC_Byte_start + 1]);
		  sprintf((char *)tx_buffer,"Stored CRC: %x\n\rCalculated CRC: %x\n\r",stored_crc,cal_crc);
		  putMessages(tx_buffer);
		  if(cal_crc != stored_crc)
		  {
			  //throw error condition

			  putMessages((uint8_t*)"CRC Error!!!\n\r");

			  putMessages((uint8_t*)"Press BLACK PUSHBUTTON for starting UART Rx interrupt again\n\r");
			  putMessages((uint8_t*)"===============================END==========================\n\r");
			  re_init_rx = SET;
		  }else{
			  //jump to the application
			  putMessages((uint8_t*)"CRC Matched, jumping to application\n\r");

			  putMessages((uint8_t*)"Press BLACK PUSHBUTTON for starting UART Rx interrupt again\n\r");
			  putMessages((uint8_t*)"===============================END==========================\n\r");
			  re_init_rx = SET;
		  }
	  }

	  if(re_init_rx)
	  {
		  re_init_rx = RESET;
		  putMessages((uint8_t*)"Re-Initiated Rx Interrupt\n\r");

#if defined(SHOW_UART2_STATUS)
		  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, UART2_RX_LENGTH_IN_BYTES);
		  switch(status)
		  {
		  case HAL_OK:
			  putMessages((uint8_t*)"HAL OK\n\r");
			  break;
		  case HAL_ERROR:
			  putMessages((uint8_t*)"HAL ERROR\n\r");
			  break;
		  case HAL_BUSY:
			  putMessages((uint8_t*)"HAL BUSY\n\r");
			  break;
		  case HAL_TIMEOUT:
			  putMessages((uint8_t*)"HAL TIMEOUT\n\r");
			  break;
		  }
#else
		  HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, UART2_RX_LENGTH_IN_BYTES);
#endif
	  }
#else
	  if(UART_Rx_Complete)
	  {
		//RESET Flag
		UART_Rx_Complete = RESET;

		//Data received from UART
		//putMessages((uint8_t*)"Received data from UART\n\r");
		live_data_row += 1;
		//Write data into FLASH
		//WriteDATAintoFlash(FLASHStartingAddress, UART_Buffer.UART_Rx_buffer, data_bytes);
		WriteByteintoFlash(FLASHStartingAddress, UART_Buffer.UART_Rx_buffer, data_bytes);
		if(recv_packet_counter >= no_of_rows - 1)//410-1)
		{
			//Data received from UART
			putMessages((uint8_t*)"Jumping to application\n\r");
			//Jump to the aplication
//			go2App();
			HAL_NVIC_SystemReset();
		}else
			//reset the UART for 5 no of bytes
			HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, (uint16_t)5);
	  }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;//38400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void FOTA_BoorloaderStateMachine(void)
{
	/*Local variable*/
	bootloader_handle_t BOOTLOADER_HANLDE_STRUCT;
	uint32_t local_boot_sequence = 0;
	uint32_t local_boot_address = 0;

	/*Set the memory location to zero*/
	memset(&BOOTLOADER_HANLDE_STRUCT,0,sizeof(BOOTLOADER_HANLDE_STRUCT));

	/*Read the configuration data from FLASH 0x8010000U*/
	FlashRead(CONFIG_DATA_ADDRESS, BOOTLOADER_HANLDE_STRUCT.u32array, sizeof(BOOTLOADER_HANLDE_STRUCT.u8array)/4);

	/*Get BOOT sequence*/
	local_boot_sequence = BOOTLOADER_HANLDE_STRUCT.BOOTLOADER_CONFIG_DATA.BOOT_SEQUENCE;

	/*Switch cases for boot-loader*/
	switch(local_boot_sequence)
	{
	case e_BOOT_ACTIVE:
		local_boot_address = BOOTLOADER_HANLDE_STRUCT.ACTIVE_PROGRAM.program_ResetHandler_address;
		break;

	case e_BOOT_BACKUP:
		local_boot_address = BOOTLOADER_HANLDE_STRUCT.BACKUP_PROGRAM.program_ResetHandler_address;
		break;
	}

	/*Jump to the application*/
	go2App(local_boot_address);

}
uint16_t crc_calc(char* input_str, int len )
{
    int pos = 0,i = 0;
    uint16_t crc1 = CRC_START_MODBUS;
    for (pos = 0; pos < len; pos++)
    {
        crc1 ^= (uint16_t)input_str[pos];            // XOR byte into least significant byte of CRC
        for (i = 8; i != 0; i--)
        {                                               // Loop over each bit
            if ((crc1 & 0x0001) != 0)
            {                                           // If the LSB is set
                crc1 >>= 1;                              // Shift right and XOR 0xA001
                crc1 ^= CRC_POLY_16;
            }
            else
            {                                           // Else LSB is not set
                crc1 >>= 1;
            }                                           // Just shift right
        }
    }
    return crc1;
}

void configuration_data(void)
{
	/*Local variable*/
	bootloader_handle_t BOOTLOADER_HANLDE_STRUCT;
	uint32_t lu16_crc_16_bits = 0;
	uint16_t lu16_cal_crc_16_bits = 0;

	/*Set the memory location to zero*/
	memset(&BOOTLOADER_HANLDE_STRUCT,0,sizeof(BOOTLOADER_HANLDE_STRUCT));

	/*Read the configuration data from FLASH 0x8000000U*/
	FlashRead(CONFIG_DATA_ADDRESS, BOOTLOADER_HANLDE_STRUCT.u32array, sizeof(BOOTLOADER_HANLDE_STRUCT.u8array)/4);

	/*Read the crc from flash*/
	lu16_crc_16_bits = BOOTLOADER_HANLDE_STRUCT.crc_16_bits;

	//uint16_t temp = sizeof(BOOTLOADER_HANLDE_STRUCT);
	/*Calculate the crc of the data from flash*/
	lu16_cal_crc_16_bits = crc_calc((char *)BOOTLOADER_HANLDE_STRUCT.u8array, sizeof(BOOTLOADER_HANLDE_STRUCT.u8array) - LENGTH_OF_CRC_IN_BYTES);

	/*Compare the stored and calculated CRC*/
	if(lu16_cal_crc_16_bits == lu16_crc_16_bits)
	{
		/*Data is valid*/
		BOOTLOADER_STATUSBITS.data_valid = SET;
	}else
	{
		/*Data is invalid load*/
		BOOTLOADER_STATUSBITS.data_invalid = SET;
		/*Check for memory corruption*/
		if(!strcmp((const char *)BOOTLOADER_HANLDE_STRUCT.reboot_string,(const char *)DEAFBEEF_STRING))
		{
			//there is memory corruption
			BOOTLOADER_STATUSBITS.data_corrupt = SET;
			//Raise flag for configuration memory corruption
#if (LOAD_DEFAULT_CONFIG_TEST == 1)

			//erase the program partition sectors (Program A)
			erase_program_partition_sectors(5);
			erase_program_partition_sectors(6);
			erase_program_partition_sectors(7);
			erase_program_partition_sectors(8);

			//load default data
			load_default_confif_data_in_flash();
#endif

		}else
		{
			//first time boot
			BOOTLOADER_STATUSBITS.first_boot = SET;
			//load default data
			load_default_confif_data_in_flash();

		}
	}
}

static void load_default_confif_data_in_flash(void)
{
	/*Local variable*/
	bootloader_handle_t BOOTLOADER_HANLDE_STRUCT;
	uint16_t lu16_cal_crc_16_bits = 0;

	/*Set the memory location to zero*/
	memset(&BOOTLOADER_HANLDE_STRUCT.u8array,0,sizeof(BOOTLOADER_HANLDE_STRUCT.u8array));

	//BOOTLOADER_HANLDE_STRUCT.ACTIVE_PROGRAM.program_ResetHandler_address = PARTION_A_START_ADDRESS;
	memcpy(BOOTLOADER_HANLDE_STRUCT.reboot_string,DEAFBEEF_STRING,sizeof(DEAFBEEF_STRING));

	//Calculate CRC
	lu16_cal_crc_16_bits = crc_calc((char *)BOOTLOADER_HANLDE_STRUCT.u8array,
			sizeof(BOOTLOADER_HANLDE_STRUCT.u8array) - LENGTH_OF_CRC_IN_BYTES);

	BOOTLOADER_HANLDE_STRUCT.crc_16_bits = lu16_cal_crc_16_bits;

	//erase the setor
	erase_program_partition_sectors(3);

	//Write the structure in the FLASH
	WriteDATAintoFlash(CONFIG_DATA_ADDRESS, BOOTLOADER_HANLDE_STRUCT.u32array, sizeof(BOOTLOADER_HANLDE_STRUCT.u32array)/4);


}
static void erase_program_partition_sectors(uint8_t sector)
{
	EraseFlashSector(sector);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
