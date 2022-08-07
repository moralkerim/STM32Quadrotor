/*
 * FLASH_PAGE.h
 *
 *  Created on: Aug 4, 2022
 *      Author: Bader
 */


#ifndef INC_FLASH_PAGE_F1_H_
#define INC_FLASH_PAGE_F1_H_
#define UAV_ID_ADDR					0x0801F800
#define CALIB_DATA_ADDR				0x0801FC00
#define WORD_SIZE				4
#include "stm32f1xx_hal.h"


uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Convert_To_Str (uint32_t *Data, char *Buf);

void Flash_Write_NUM (uint32_t StartSectorAddress, float *Num, int numwords);

float Flash_Read_NUM (uint32_t StartSectorAddress);



/********************  FLASH_Error_Codes   ***********************//*
HAL_FLASH_ERROR_NONE      0x00U  // No error
HAL_FLASH_ERROR_PROG      0x01U  // Programming error
HAL_FLASH_ERROR_WRP       0x02U  // Write protection error
HAL_FLASH_ERROR_OPTV      0x04U  // Option validity error
*/


#endif /* INC_FLASH_PAGE_F1_H_ */
