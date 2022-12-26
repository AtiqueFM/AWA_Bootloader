/*
 * FLash.h
 *
 *  Created on: Sep 21, 2022
 *      Author: atiqueshaikh
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"

void EraseFlashSector(uint32_t Sector);

void SetFlash(uint32_t Address,uint32_t flag);

void WriteDATAintoFlash(uint32_t Address,uint8_t* pData,uint32_t wordLength);

void FlashRead(uint32_t Address,uint8_t *pData,uint32_t numberofwords);
#endif /* INC_FLASH_H_ */
