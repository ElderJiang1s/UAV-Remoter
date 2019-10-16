#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H
#include "NuMicro.h"

void SPI_FLASH_Init(void);


u8 SPIM_SendByte(u8 data);

u32 SPI_FLASH_ReadID(void);
void SPI_WaitForWriteEnd(void);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WriteDisable(void);
void SPI_Flash_Read_Data(uint8_t *readBuff,uint32_t addr,uint32_t numByteToRead);
u8 SPI_Flash_ReadSR(void);
void SPI_FLASH_Write_SR(u8 sr);
void SPI_Flash_Erase_Chip(void);
void SPI_Flash_Erase_Sector(u32 Dst_Addr);
void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);




#endif

