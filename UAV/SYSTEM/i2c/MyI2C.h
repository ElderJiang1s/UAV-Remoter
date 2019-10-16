#ifndef __SOFTWAREI2C_H
#define __SOFTWAREI2C_H
#include "NuMicro.h"

#define SCL PB11
#define SDA PB10

#define SI2C

void SI2C_Start(void);
void SI2C_Stop(void);
void SI2C_Ack(void);
void SI2C_NAck(void);
u8 SI2C_WaitAck(void);
void SI2C_SendByte(u8 data);
u8 SI2C_ReadByte(u8 ack);








void MyI2C_Init(void);


u8 MyI2C_WriteByte(u8 addr,u8 reg,u8 data);
u8 MyI2C_ReadByte(u8 addr,u8 reg);
u8 I2C_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 I2C_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
#endif
