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
#define CONFIG_DATA_ADDRESS			(uint32_t)0x800c000UL			/*Flash address for configuration data*/
#define LENGTH_OF_CRC_IN_BYTES		4
#define DEAFBEEF_STRING				"DEADBEEF"
#define PARTION_A_START_ADDRESS		(uint32_t)0x8004000UL
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

	struct{
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
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
