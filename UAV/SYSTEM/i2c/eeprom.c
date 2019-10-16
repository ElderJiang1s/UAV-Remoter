#include "eeprom.h"


void EEPROM_Init(void)
{
	MyI2C_Init();
}

//��EEPROMָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 EEPROM_ReadByte(u8 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    SI2C_Start();  
SI2C_SendByte(0XA0);   //����������ַ0XA0,д���� 	 
	if(SI2C_WaitAck()){return 0;} 
    SI2C_SendByte(ReadAddr);   //���͵͵�ַ
	if(SI2C_WaitAck()){return 0;}	    
	SI2C_Start();  	 	   
	SI2C_SendByte(0XA1);           //�������ģʽ			   
	if(SI2C_WaitAck()){return 0;}	 
    temp=SI2C_ReadByte(0);		   
    SI2C_Stop();//����һ��ֹͣ����	    
	return temp;
}
//��EEPROMָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void EEPROM_WriteByte(u8 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    SI2C_Start();  
		SI2C_SendByte(0XA0);   //����������ַ0XA0,д���� 	 
	if(SI2C_WaitAck()){return;}	   
    SI2C_SendByte(WriteAddr);   //���͵͵�ַ
	if(SI2C_WaitAck()){return;} 	 										  		   
	SI2C_SendByte(DataToWrite);     //�����ֽ�							   
	if(SI2C_WaitAck()){return;}  		    	   
    SI2C_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//��EEPROM�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void EEPROM_WriteLenByte(u8 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		EEPROM_WriteByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//��EEPROM�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���EEPROM�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 EEPROM_Check(void)
{
	u8 temp;
	temp=EEPROM_ReadByte(255);//����ÿ�ο�����дEEPROM			   
	if(temp==0X55)return 0;		   
	else//�ų���һ�γ�ʼ�������
	{
		EEPROM_WriteByte(255,0X55);
	    temp=EEPROM_ReadByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//��EEPROM�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void EEPROM_Read(u8 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=EEPROM_ReadByte(ReadAddr++);	
		NumToRead--;
	}
}  
//��EEPROM�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
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
 



