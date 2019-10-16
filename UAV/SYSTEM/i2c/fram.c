#include "fram.h"


void FRAM_Init(void)
{
	MyI2C_Init();
}



//��FRAMָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 FRAM_ReadByte(u8 addr_msb,u8 ReadAddr)
{				  
	u8 temp=0;		
    SI2C_Start();  
SI2C_SendByte(0XA0|FRAM_WRITE|(addr_msb<<1));   //����������ַ0XA0,д���� 	 
//	if(SI2C_WaitAck()){return 0;} 
	SI2C_WaitAck();
    SI2C_SendByte(ReadAddr);   //���͵͵�ַ
SI2C_Start();  	 	   
SI2C_Start(); 
	SI2C_SendByte(0XA0|FRAM_READ|(addr_msb<<1));           //�������ģʽ			   
//	if(SI2C_WaitAck()){return 0;}	    	   
	SI2C_WaitAck();
    temp=SI2C_ReadByte(0);		   
    SI2C_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��FRAMָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void FRAM_WriteByte(u8 addr_msb,u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    SI2C_Start();  
		SI2C_SendByte(0XA0|FRAM_WRITE|(addr_msb<<1));   //����������ַ0XA0,д���� 	 
//	if(SI2C_WaitAck()){return;}	   
	SI2C_WaitAck();
    SI2C_SendByte(WriteAddr);   //���͵͵�ַ
//	if(SI2C_WaitAck()){return;} 	 
SI2C_WaitAck();	
	SI2C_SendByte(DataToWrite);     //�����ֽ�							   
//	if(SI2C_WaitAck()){return;}  	
SI2C_WaitAck();	
    SI2C_Stop();//����һ��ֹͣ���� 
	//delay_ms(10);	 
}





//��FRAM�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void FRAM_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		FRAM_WriteByte(0,WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��FRAM�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���FRAM�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 FRAM_Check(void)
{
	u8 temp;
	temp=FRAM_ReadByte(1,255);//����ÿ�ο�����дFRAM			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		FRAM_WriteByte(1,255,0X55);
	    temp=FRAM_ReadByte(1,255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��FRAM�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ 
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void FRAM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=FRAM_ReadByte(0,ReadAddr++);	
		NumToRead--;
	}
}  
//��FRAM�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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
 

