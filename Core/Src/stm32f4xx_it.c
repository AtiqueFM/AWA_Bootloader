/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t recv_packet_counter;
uint8_t temp_buf[100];
uint8_t data_frame;
uint8_t data_bytes;
uint32_t FLASHStartingAddress;
uint32_t file_size_in_bytes = 0;
extern uint8_t file_info_flag;
extern uint32_t no_of_rows;

extern Buffer_t UART_Buffer;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)
	{
		//UART_Rx_Complete = SET;
		//Abort UART 2 Rx
		//HAL_UART_Abort_IT(&huart6);

		HAL_UART_AbortReceive_IT(&huart6);
//		recv_packet_counter += 1;
		if(file_info_flag < 3)
			file_info_flag += 1;
		if(file_info_flag > 2)
		{
			data_frame += 1;
			switch(data_frame)
			{
			case 1:
			{
				//Abort the RX and restart the Rx for no of data bytes
				HAL_UART_AbortReceive_IT(&huart6);
				//received the 5 bytes
				//Get flash address
				FLASHStartingAddress = (UART_Buffer.UART_Rx_buffer[0]<<24)
						|(UART_Buffer.UART_Rx_buffer[1]<<16)
						|(UART_Buffer.UART_Rx_buffer[2]<<8)
						|(UART_Buffer.UART_Rx_buffer[3]);
				//Printing flash starting address
				//sprintf((char*)temp_buf,"starting flash address : %lx\n\r",FLASHStartingAddress);
				//get the no of data bytes to be received
				data_bytes = UART_Buffer.UART_Rx_buffer[4];

				HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, (uint16_t)data_bytes);
				break;
			}
			case 2:
			{
				//received the n number of bytes
				//reset the data_frame to 0
				data_frame = 0;
				//abort the uart
				HAL_UART_AbortReceive_IT(&huart6);

				UART_Rx_Complete = SET;

	//			//reset the UART for 5 no of bytes
	//			HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, (uint16_t)5);
				recv_packet_counter += 1;

				//putMessages((uint8_t*)temp_buf);
				//sprintf((char*)temp_buf,"Packet : %d\n\r",recv_packet_counter);
				//putMessages((uint8_t*)temp_buf);
				break;
			}
			}
	//		uint32_t temp = huart6.Instance->SR;
	//		sprintf((char*)temp_buf,"Packet : %d\n\r",recv_packet_counter);
	//		putMessages((uint8_t*)temp_buf);
		}else
		{
			switch(file_info_flag)
			{
			case 1:
				//Number of rows in file
				//Abort the RX and restart the Rx for no of data bytes
				HAL_UART_AbortReceive_IT(&huart6);
				//Get the file info
				no_of_rows = (UART_Buffer.UART_Rx_buffer[0]<<24)
						|(UART_Buffer.UART_Rx_buffer[1]<<16)
						|(UART_Buffer.UART_Rx_buffer[2]<<8)
						|(UART_Buffer.UART_Rx_buffer[3]);
				//sprintf((char*)temp_buf,"Number of rows in file : %ld\n\r",no_of_rows);
				//putMessages((uint8_t*)temp_buf);

				HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, 4);
				break;
			case 2:
				//File size in bytes
				//Abort the RX and restart the Rx for no of data bytes
				HAL_UART_AbortReceive_IT(&huart6);
				//Get the file info
				file_size_in_bytes = (UART_Buffer.UART_Rx_buffer[0]<<24)
						|(UART_Buffer.UART_Rx_buffer[1]<<16)
						|(UART_Buffer.UART_Rx_buffer[2]<<8)
						|(UART_Buffer.UART_Rx_buffer[3]);
				//sprintf((char*)temp_buf,"Number of rows in file : %ld\n\r",file_size_in_bytes);
				//putMessages((uint8_t*)temp_buf);
				HAL_UART_Receive_IT(&huart6, UART_Buffer.UART_Rx_buffer, 5);
				file_info_flag = 3;
				break;
			default:
				break;
			}
		}


	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
