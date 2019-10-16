#include "eeprom.h"


void EEPROM_Init(void)
{
	MyI2C_Init();
}

//在EEPROM指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 EEPROM_ReadByte(u8 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    SI2C_Start();  
SI2C_SendByte(0XA0);   //发送器件地址0XA0,写数据 	 
	if(SI2C_WaitAck()){return 0;} 
    SI2C_SendByte(ReadAddr);   //发送低地址
	if(SI2C_WaitAck()){return 0;}	    
	SI2C_Start();  	 	   
	SI2C_SendByte(0XA1);           //进入接收模式			   
	if(SI2C_WaitAck()){return 0;}	 
    temp=SI2C_ReadByte(0);		   
    SI2C_Stop();//产生一个停止条件	    
	return temp;
}
//在EEPROM指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void EEPROM_WriteByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    SI2C_Start();  
		SI2C_SendByte(0XA0);   //发送器件地址0XA0,写数据 	 
	if(SI2C_WaitAck()){return;}	   
    SI2C_SendByte(WriteAddr);   //发送低地址
	if(SI2C_WaitAck()){return;} 	 										  		   
	SI2C_SendByte(DataToWrite);     //发送字节							   
	if(SI2C_WaitAck()){return;}  		    	   
    SI2C_Stop();//产生一个停止条件 
	delay_ms(10);	 
}
//在EEPROM里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void EEPROM_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		EEPROM_WriteByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在EEPROM里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 EEPROM_ReadLenByte(u8 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=EEPROM_ReadByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查EEPROM是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 EEPROM_Check(void)
{
	u8 temp;
	temp=EEPROM_ReadByte(255);//避免每次开机都写EEPROM			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		EEPROM_WriteByte(255,0X55);
	    temp=EEPROM_ReadByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在EEPROM里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void EEPROM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=EEPROM_ReadByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在EEPROM里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void EEPROM_Write(u8 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	u8 temp;
	while(NumToWrite--)
	{
		EEPROM_WriteByte(WriteAddr,*pBuffer);
		temp=EEPROM_ReadByte(WriteAddr);
		if(temp != *pBuffer)
		{
			while(1)
			{
				led1=1;
				delay_ms(100);
				led1=0;
				delay_ms(100);
				led1=1;
				delay_ms(500);
				led1=0;
				delay_ms(500);
			}
		}
		WriteAddr++;
		pBuffer++;
	}
	
}
 



