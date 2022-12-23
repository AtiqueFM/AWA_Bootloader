/*
 * Application.c
 *
 *  Created on: Sep 21, 2022
 *      Author: atiqueshaikh
 */


#include "main.h"
#include "FLash.h"


/*Extern parameter defination*/
extern Buffer_t UART_Buffer;
extern Buffer_t FLash_UART_read_buffer;
//extern UART_HandleTypeDef huart3;

void EraseApplicationMemory(void)
{
	/*Erase the sector where the HEX data is to be stored*/
	EraseFlashSector((uint32_t)HEXFILE_FLASHSECTOR);
}
#if 0
void WriteHEXFILEtoFLash(void)
{
	/*Erase the sector where the HEX data is to be stored*/
	EraseFlashSector((uint32_t)HEXFILE_FLASHSECTOR);

	/*Write the HEX data into flash*/
	WriteDATAintoFlash(HEXFILE_FLASHADDRESS, UART_Buffer.UART_Rx_Word_buffer, (uint32_t)UART2_RX_LENGTH_IN_BYTES);
}
#endif
void ReadHEXFILEfromFlash(void)
{
	FlashRead(HEXFILE_FLASHADDRESS, FLash_UART_read_buffer.UART_Rx_Word_buffer, BLE_DATA_RX_IN_WORDS);
}

void putMessages(uint8_t *pData)
{
#if 0
	uint32_t len = 0;
	//temp pointer
	uint8_t *temp = NULL;
	//copy the address to the temp pointer
	temp = pData;
	//find the length of the string
	while(*temp != '\0')
	{
		len += 1;
		temp += 1;
	}
	//print the string
	HAL_UART_Transmit(&huart3, pData, len, 1000);
#endif
}

