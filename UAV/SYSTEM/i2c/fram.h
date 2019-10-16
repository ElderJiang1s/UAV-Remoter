#ifndef __FRAM_H
#define __FRAM_H
#include "NuMicro.h"
#include "delay.h"
#include "MyI2C.h"
#define FRAM_READ 	1
#define FRAM_WRITE 	0

void FRAM_Init(void);
u8 FRAM_ReadByte(u8 addr_msb,u8 ReadAddr);
void FRAM_WriteByte(u8 addr_msb,u8 WriteAddr,u8 DataToWrite);
void FRAM_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len);
u32 FRAM_ReadLenByte(u8 ReadAddr,u8 Len);
u8 FRAM_Check(void);
void FRAM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead);
void FRAM_Write(u8 WriteAddr,u8 *pBuffer,u16 NumToWrite);




#endif

