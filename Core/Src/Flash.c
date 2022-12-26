/*
 * Flash.c
 *
 *  Created on: Sep 21, 2022
 *      Author: atiqueshaikh
 */
#include "main.h"

#include "FLash.h"
#include "string.h"

void EraseFlashSector(uint32_t Sector)
{
	//unlock the flash action
	HAL_FLASH_Unlock();
	/*		FLASH ERASE			*/
	FLASH_Erase_Sector(Sector,FLASH_VOLTAGE_RANGE_3);
	//Lock the flash action
	HAL_FLASH_Lock();

}
void SetFlash(uint32_t Address,uint32_t flag)
{
	//unlock the flash action
	HAL_FLASH_Unlock();
	/*		DATA WRITE			*/
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, flag);
	/**********************************/
	//Lock the flash action
	HAL_FLASH_Lock();


}


void WriteDATAintoFlash(uint32_t Address,uint32_t* pData,uint32_t wordLength)

{
	//Local variables
	uint32_t index = 0;
	//unlock the flash action
	HAL_FLASH_Unlock();
	/*		DATA WRITE			*/
	while(index < wordLength)
	{

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, pData[index]) == HAL_OK)

		{

			//Increment the flash address by 4bytes for 32 bits of data is being stored
			Address += 4;

			//Increment the index by one as the data buffer is of 32 bits
			index += 1;
		}
		else{
			putMessages((uint8_t*)"Error in flash writing\n\r");
		}
	}
	/**********************************/
	//Lock the flash action
	HAL_FLASH_Lock();
}

void WriteByteintoFlash(uint32_t Address,uint8_t* pData, uint32_t wordlength)
{
	//Local variables
	uint32_t index = 0;
	//unlock the flash action
	HAL_FLASH_Unlock();
	/*		DATA WRITE			*/
	while(index < wordlength)
	{

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, pData[index]) == HAL_OK)

		{

			//Increment the flash address by 1bytes for 8 bits of data is being stored
			Address += 1;
			//Increment the index by one as the data buffer is of 32 bits
			index += 1;
		}
		else{
			putMessages((uint8_t*)"Error in flash writing\n\r");
		}
	}
	/**********************************/
	//Lock the flash action
	HAL_FLASH_Lock();
}

void FlashRead(uint32_t Address,uint32_t *pData,uint32_t numberofwords)
{
	uint32_t StartPageAddress = 0;
	StartPageAddress = Address;
	while(1)
	{
		*pData = *(uint32_t *)StartPageAddress;
		pData += 1;
		StartPageAddress += 4;
		if (!(numberofwords--)) break;
	}

}
