#include "MyI2C.h"
//PB10 I2C1_SDA
//PB11 I2C1_SCL

void MyI2C_Init(void)
{
#ifdef SI2C
	GPIO_SetMode(PB,BIT10,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PB,BIT11,GPIO_MODE_OUTPUT);
#else
	SYS_UnlockReg();
	CLK_SetModuleClock(I2C1_MODULE,0,0);
	CLK_EnableModuleClock(I2C1_MODULE);
	SYS->GPB_MFPH|=SYS_GPB_MFPH_PB10MFP_I2C1_SDA|SYS_GPB_MFPH_PB11MFP_I2C1_SCL;
	I2C_Open(I2C1,400000);
	SYS_LockReg();
#endif
}

#ifdef SI2C
static void i2c_Delay(void)
{
	uint8_t i;
	for (i=0;i<15;i++);
}


void SDA_IN(void)
{
	GPIO_SetMode(PB,BIT10,GPIO_MODE_INPUT);
}

void SDA_OUT(void)
{
	GPIO_SetMode(PB,BIT10,GPIO_MODE_OPEN_DRAIN);
}

void SI2C_Start(void)//SCLΪ1ʱ��SDA��1��0���䣬��ʼ��������
{
	SDA_OUT();
	SDA=1;
	SCL=1;
	i2c_Delay();
	SDA=0;
	i2c_Delay();
	SCL=0;
}

void SI2C_Stop(void)//SCLΪ1ʱ��SDA��0��1���䣬������������
{
	SDA_OUT();
	SCL=0;
	SDA=0;
	i2c_Delay();/**/
	SCL=1;
	SDA=1;
	i2c_Delay();
}

void SI2C_Ack(void)
{
	SCL=0;
	SDA_OUT();
	SDA=0;
	i2c_Delay();
	SCL=1;
	i2c_Delay();
	SCL=0;	
}

void SI2C_NAck(void)
{
	SCL=0;
	SDA_OUT();
	SDA=1;
	i2c_Delay();
	SCL=1;
	i2c_Delay();
	SCL=0;
}

u8 SI2C_WaitAck(void)//1������Ӧ��ʧ��	0������Ӧ��ɹ�
{
	u16 temp;
	SDA_IN();
	SDA=1;
	i2c_Delay();/**/
	SCL=1;
	i2c_Delay();/**/
	while(SDA)
	{
		temp++;
		if(temp>1000)
		{
			SI2C_Stop();
			return(1);
		}
	}
	SCL=0;
	return(0);
}

void SI2C_SendByte(u8 data)
{
	u8 t;
	SDA_OUT();
	SCL=0;
	for(t=0;t<8;t++)
	{
		SDA=(data&0x80)>>7;
		data<<=1;
		i2c_Delay();
		SCL=1;
		i2c_Delay();
		SCL=0;
		i2c_Delay();
	}
}

u8 SI2C_ReadByte(u8 ack)//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
{
	u8 i,temp=0;
	SDA_IN();
	for(i=0;i<8;i++)
	{
		SCL=0;
		i2c_Delay();
		SCL=1;
		temp<<=1;
		if(SDA)
		{temp++;}
		i2c_Delay();
	}
	if(!ack)
	{
		SI2C_NAck();
	}else{
		SI2C_Ack();
	}
	return(temp);
}
#endif

u8 MyI2C_WriteByte(u8 addr,u8 reg,u8 data)
{
#ifdef SI2C
	SI2C_Start();
	SI2C_SendByte((addr<<1)|0);//MPU6050��ַ��0λ1����  0��д
	if(SI2C_WaitAck())
	{
		SI2C_Stop();//��ʱ
		return(1);
	}
	SI2C_SendByte(reg);
	if(SI2C_WaitAck())
	{
		SI2C_Stop();//��ʱ
		return(1);
	}
	SI2C_SendByte(data);
	if(SI2C_WaitAck())
	{
		SI2C_Stop();//��ʱ
		return(1);
	}
	SI2C_Stop();
	return(0);
#else
	I2C_WriteByteOneReg(I2C1, addr, reg, data);
	I2C_STOP(I2C1);
	return(0);
#endif
}

u8 MyI2C_ReadByte(u8 addr,u8 reg)
{
#ifdef SI2C
	u8 temp;
	SI2C_Start();
	SI2C_SendByte((addr<<1)|0);//MPU6050��ַ��0λ1����  0��д
	SI2C_WaitAck();
	SI2C_SendByte(reg);
	SI2C_WaitAck();
	SI2C_Start();
	SI2C_SendByte((addr<<1)|1);
	SI2C_WaitAck();
	temp=SI2C_ReadByte(0);//����nAck
	SI2C_Stop();
	return(temp);
#else
	u8 temp;
	temp=I2C_ReadByteOneReg(I2C1, addr, reg);
	I2C_STOP(I2C1);
	return(temp);
#endif
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 I2C_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
#ifdef SI2C
	u8 i; 
	SI2C_Start();
	SI2C_SendByte((addr<<1)|0);//����������ַ+д����	
	if(SI2C_WaitAck())	//�ȴ�Ӧ��
	{
		SI2C_Stop();		 
		return 1;		
	}
	SI2C_SendByte(reg);	//д�Ĵ�����ַ
    SI2C_WaitAck();	//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		SI2C_SendByte(buf[i]);	//��������
		if(SI2C_WaitAck())		//�ȴ�ACK
		{
			SI2C_Stop();	 
			return 1;		 
		}		
	}    
	SI2C_Stop();
	return 0;	
#else
I2C_WriteMultiBytesOneReg(I2C1, addr, reg, buf, len);
	I2C_STOP(I2C1);
	return 0;
#endif
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 I2C_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
#ifdef SI2C
	SI2C_Start();
	SI2C_SendByte((addr<<1)|0);//����������ַ+д����	
	if(SI2C_WaitAck())	//�ȴ�Ӧ��
	{
		SI2C_Stop();	 
		return 1;		
	}
	SI2C_SendByte(reg);	//д�Ĵ�����ַ
    SI2C_WaitAck();		//�ȴ�Ӧ��
	SI2C_Start();
	SI2C_SendByte((addr<<1)|1);//����������ַ+������	
    SI2C_WaitAck();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=SI2C_ReadByte(0);//������,����nACK 
		else *buf=SI2C_ReadByte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
   SI2C_Stop();	//����һ��ֹͣ���� 
	return 0;	
#else
	I2C_ReadMultiBytesOneReg(I2C1, addr, reg, buf, len);
	I2C_STOP(I2C1);
	return 0;	
#endif
}

