#ifndef __EEPROM_H
#define __EEPROM_H
#include "NuMicro.h"
#include "delay.h"
#include "MyI2C.h"
#define addr_24c02 0x50

void EEPROM_Init(void);
u8 EEPROM_ReadByte(u8 ReadAddr);
void EEPROM_WriteByte(u8 WriteAddr,u8 DataToWrite);

void EEPROM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead);
void EEPROM_Write(u8 WriteAddr,u8 *pBuffer,u16 NumToWrite);

#endif

