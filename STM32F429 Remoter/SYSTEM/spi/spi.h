#ifndef __SPI1_H
#define __SPI1_H
#include "stm32f10x.h"
#include "sys.h"

void SPI1_Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI_SendByte(u8 data);
u32 SPI_FLASH_ReadID(void);//SPI_FLASH
void SPI_WaitForWriteEnd(void);//SPI_FLASH
void SPI_FLASH_WriteEnable(void);//SPI_FLASH
void SPI_FLASH_WriteDisable(void);//SPI_FLASH
void SPI_Write_Data(uint32_t addr,uint8_t *writeBuff,uint32_t numByteToWrite);//SPI_FLASH
void SPI_Flash_Read_Data(uint32_t addr,uint8_t *readBuff,uint32_t numByteToRead);//SPI FLASH

#endif
