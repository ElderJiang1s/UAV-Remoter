#include "fram.h"


void FRAM_Init(void)
{
	MyI2C_Init();
}



//在FRAM指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 FRAM_ReadByte(u8 addr_msb,u8 ReadAddr)
{				  
	u8 temp=0;		
    SI2C_Start();  
SI2C_SendByte(0XA0|FRAM_WRITE|(addr_msb<<1));   //发送器件地址0XA0,写数据 	 
//	if(SI2C_WaitAck()){return 0;} 
	SI2C_WaitAck();
    SI2C_SendByte(ReadAddr);   //发送低地址
SI2C_Start();  	 	   
SI2C_Start(); 
	SI2C_SendByte(0XA0|FRAM_READ|(addr_msb<<1));           //进入接收模式			   
//	if(SI2C_WaitAck()){return 0;}	    	   
	SI2C_WaitAck();
    temp=SI2C_ReadByte(0);		   
    SI2C_Stop();//产生一个停止条件	    
	return temp;
}
//在FRAM指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void FRAM_WriteByte(u8 addr_msb,u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    SI2C_Start();  
		SI2C_SendByte(0XA0|FRAM_WRITE|(addr_msb<<1));   //发送器件地址0XA0,写数据 	 
//	if(SI2C_WaitAck()){return;}	   
	SI2C_WaitAck();
    SI2C_SendByte(WriteAddr);   //发送低地址
//	if(SI2C_WaitAck()){return;} 	 
SI2C_WaitAck();	
	SI2C_SendByte(DataToWrite);     //发送字节							   
//	if(SI2C_WaitAck()){return;}  	
SI2C_WaitAck();	
    SI2C_Stop();//产生一个停止条件 
	//delay_ms(10);	 
}





//在FRAM里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void FRAM_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		FRAM_WriteByte(0,WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在FRAM里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 FRAM_ReadLenByte(u8 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=FRAM_ReadByte(0,ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查FRAM是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 FRAM_Check(void)
{
	u8 temp;
	temp=FRAM_ReadByte(1,255);//避免每次开机都写FRAM			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		FRAM_WriteByte(1,255,0X55);
	    temp=FRAM_ReadByte(1,255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在FRAM里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void FRAM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=FRAM_ReadByte(0,ReadAddr++);	
		NumToRead--;
	}
}  
//在FRAM里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void FRAM_Write(u8 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	u8 temp;
	while(NumToWrite--)
	{
		FRAM_WriteByte(0,WriteAddr,*pBuffer);
		temp=FRAM_ReadByte(0,WriteAddr);
		if(temp != *pBuffer)
		{
			__ASM("nop");
//			while(1)
//			{
//				led1=1;
//				delay_ms(100);
//				led1=0;
//				delay_ms(100);
//				led1=1;
//				delay_ms(500);
//				led1=0;
//				delay_ms(500);
//			}
		}
		WriteAddr++;
		pBuffer++;
	}
	
}
 

