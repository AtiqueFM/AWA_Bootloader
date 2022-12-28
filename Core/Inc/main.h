/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*TEST FLAGS*/
#define LOAD_DEFAULT_CONFIG_TEST	1
/*Code version*/
#define MAJOR_VER					2
#define MINOR_VER					0
#define BUG_FIX						0
/*Date of last modification*/
#define DD							26
#define MM							12
#define YY							22

#define UART2_RX_LENGTH_IN_BYTES	400//512								/*<Warning :- Address in multiples of 4*/
#define BLE_DATA_RX_IN_WORDS		(UART2_RX_LENGTH_IN_BYTES / 4)
#define UART2_RX_TIMEOUT			1000
#define CRC_POLY_16         		0xA001
#define CRC_START_MODBUS    		0xFFFF
#define HEXFILE_FLASHSECTOR			6
#define HEXFILE_FLASHADDRESS		(uint32_t)0x8020000				/*<Address of the application*/
#define UART_BUFFER_SIZE_U8			512								/*<Warning :- Address in multiples of 4*/
#define UART_BUFFER_SIZE_U32		UART_BUFFER_SIZE_U8 / 4
#define CONFIG_DATA_ADDRESS			(uint32_t)0x8010000UL			/*Flash address for configuration data*/
#define LENGTH_OF_CRC_IN_BYTES		4
#define DEAFBEEF_STRING				"DEADBEEF"
#define PARTION_A_START_ADDRESS		(uint32_t)0x8004000UL

//Booting sequence
enum{
	e_BOOT_ACTIVE = 1,
	e_BOOT_BACKUP
}enum_bootsequence;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void FOTA_Hearbeat(void);
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t crc_calc(char* input_str, int len );

void putMessages(uint8_t *pData);

void WriteHEXFILEtoFLash(void);
void ReadHEXFILEfromFlash(void);
void EraseApplicationMemory(void);


void FOTA_BoorloaderStateMachine(void);
uint8_t UART_Rx_Complete;
uint16_t Rx_Counter;
enum{
	CRC_Byte_start = 398,
};

typedef union
{
	uint8_t UART_Rx_buffer[UART_BUFFER_SIZE_U8];
	uint32_t UART_Rx_Word_buffer[UART_BUFFER_SIZE_U32];
}Buffer_t;

typedef struct __attribute__((packed))
{
	uint32_t BOOTLOADER_PARTITION_SIZE;
	uint32_t BOOT_SEQUENCE;
	uint32_t PARTION_A_RESET_HANDLER_ADDRESS;
	uint32_t PARTION_A_FLASH_SIZE;
	uint32_t PARTION_A_CCRAM_SIZE;
	uint32_t PARTION_B_RESET_HANDLER_ADDRESS;
	uint32_t PARTION_B_FLASH_SIZE;
	uint32_t PARTION_B_CCRAM_SIZE;
}bootloader_configuration_handle_t;

typedef struct __attribute__((packed)){
	uint32_t program_ResetHandler_address;
	uint32_t program_16bit_crc;
	uint32_t program_size;
	uint32_t program_partion;
	uint32_t program_loadtimestamp;
	uint32_t program_reeived_timestamp;
	uint32_t program_Firmware_version;
	uint32_t program_Author;
	uint32_t program_status_flags;
}program_configuration_hanlde_t;


typedef union __attribute__((packed)){

	uint8_t u8array[(sizeof(uint8_t) * 8)
					+ sizeof(bootloader_configuration_handle_t)
					+ sizeof(program_configuration_hanlde_t)
					+ sizeof(program_configuration_hanlde_t)
					+ sizeof(uint32_t)
					+ sizeof(uint32_t)];
	uint32_t u32array[30];

	struct __attribute__((packed)){
		uint8_t reboot_string[8];
		bootloader_configuration_handle_t BOOTLOADER_CONFIG_DATA;
		program_configuration_hanlde_t ACTIVE_PROGRAM;
		program_configuration_hanlde_t BACKUP_PROGRAM;
		uint32_t Failure_state_handle;
		uint32_t crc_16_bits;
	};
}bootloader_handle_t;


typedef struct{
	unsigned data_valid 	: 1;
	unsigned data_invalid	: 1;
	unsigned data_corrupt	: 1;
	unsigned first_boot		: 1;
}bootloader_status_hanlde_t;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
