#include "bmpxxx.h" 

#ifdef BMP180
u8 oss=0;
s16 AC1=0,AC2=0,AC3=0,B1=0,B2=0,MB=0,MC=0,MD=0;
u16 AC4=0,AC5=0,AC6=0;
#endif
#ifdef BMP280
u16 dig_T1,dig_P1;
s16 dig_T2,dig_T3,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
#endif



static u16 BMP_Read16(u8 addr)
{				  
	u16 temp=0;
	u8 tempH=0,tempL=0;
#ifdef BMP180
	tempH=MyI2C_ReadByte(BMP_I2C_Addr,addr);
	tempL=MyI2C_ReadByte(BMP_I2C_Addr,addr+1);
#endif
#ifdef BMP280
	tempL=MyI2C_ReadByte(BMP_I2C_Addr,addr);
	tempH=MyI2C_ReadByte(BMP_I2C_Addr,addr+1);
#endif
	temp=((u16)tempH<<8)|((u16)tempL&0xFF);
	return temp;
}

static u32 BMP_Read32(u8 MSBAddr)
{				  
	u32 temp=0;
	u8 MSB=0,LSB=0,XLSB=0;
	MSB=MyI2C_ReadByte(BMP_I2C_Addr,MSBAddr);
	LSB=MyI2C_ReadByte(BMP_I2C_Addr,MSBAddr+1);
	XLSB=MyI2C_ReadByte(BMP_I2C_Addr,MSBAddr+2);
	//temp=(((u32)MSB<<16)|((u32)LSB<<8)|(u32)XLSB)>>4;
	 temp = (u32)(((u32)MSB << 12)|((u32)LSB << 4)|((u32)XLSB >> 4));
	return temp;
}
static void bmp_getcalib(void)
{
	delay_ms(50);					//这个delay调小输出会不准
	#ifdef BMP180
	AC1=BMP_Read16(0XAA);
	AC2=BMP_Read16(0XAC);
	AC3=BMP_Read16(0XAE);
	AC4=BMP_Read16(0XB0);
	AC5=BMP_Read16(0XB2);
	AC6=BMP_Read16(0XB4);
	B1=BMP_Read16(0XB6);
	B2=BMP_Read16(0XB8);
	MB=BMP_Read16(0XBA);
	MC=BMP_Read16(0XBC);
	MD=BMP_Read16(0XBE);
#endif
#ifdef BMP280
	dig_T1=BMP_Read16(0X88);
	dig_T2=BMP_Read16(0X8A);
	dig_T3=BMP_Read16(0X8C);
	dig_P1=BMP_Read16(0X8E);
	dig_P2=BMP_Read16(0X90);
	dig_P3=BMP_Read16(0X92);
	dig_P4=BMP_Read16(0X94);
	dig_P5=BMP_Read16(0X96);
	dig_P6=BMP_Read16(0X98);
	dig_P7=BMP_Read16(0X9A);
	dig_P8=BMP_Read16(0X9C);
	dig_P9=BMP_Read16(0X9E);
#endif
}

u8 bmp_init(void)
{
	MyI2C_Init();
#ifdef BMP180
	if(MyI2C_ReadByte(0x77,0xD0)==0x55)
#endif
#ifdef BMP280
	if(MyI2C_ReadByte(0x76,0xD0)==0x58)
#endif
	{
#ifdef BMP280
		MyI2C_WriteByte(0x76,0xE0,0xB6);
		MyI2C_WriteByte(0x76,0xF4,0xFF);
		MyI2C_WriteByte(0x76,0xF5,0x00);
#endif
		bmp_getcalib();
		return(0);
	}else{return(1);}
}


void BMP_Calculation(float *temp,float *pres)
{
#ifdef BMP180
	s32 UT=0,UP=0;
	s32 T=0;
	s32 P=0,X1=0,X2=0,X3=0,B3=0,B5=0,B6=0,B7=0;
	u32 B4=0;
	MyI2C_WriteByte(0x77,0xF4,0x2E);
	delay_ms(6);
	UT=BMP_Read16(0xF6);	
	MyI2C_WriteByte(0x77,0xF4,0x34);
	delay_ms(7);	
	UP=BMP_Read16(0xf6);	
	
	X1 =(((s32)UT-(s32)AC6)*(s32)AC5)>>15;
	X2 = ((s32)MC<<11)/(X1+MD);
	B5 = X1 + X2;
	T = (B5 + 8)>>4;	
	*temp=T;
	
	B6 = B5 - 4000;
	X1 = (B2*(B6*B6>>12))>>11;
	X2 = AC2*B6>>11;
	X3 = X1 + X2;
	B3 = (((((s32)AC1)*4 + X3)<<oss)+2)>>2;
	//B3 = (((AC1<<2)+X3)+2)>>2;
	X1 = AC3*B6>>13;
	X2 = (B1*((B6*B6)>>12))>>16;
	X3 = ((X1+X2)+2)>>2;
	B4 = AC4*(u32)(X3 + 32768)>>15;
	B7 = ((u32)(UP - B3)*(50000>>oss));
	//B7 = (u32)(UP - B3)*50000;
	if (B7 < 0x80000000) 
	{
		P = (B7<<1)/B4;
	}
	else 
	{
		P = (B7/B4)<<1;
	}
	X1 = (P<<8)*(P<<8);
	X1 = (X1*3038)>>16;
	X2 = (-7357*P)>>16;
	P += (X1+X2+3791)>>4;
	*pres=P;
#endif
#ifdef BMP280
	s32 t_fine,adc_T,adc_P;
	float var1,var2,p,T;
	adc_T=BMP_Read32(0xFA);
	adc_P=BMP_Read32(0xF7);

	var1=(((float)adc_T)/16384.0-((float)dig_T1)/1024.0)*((float)dig_T2);
	var2=((((float)adc_T)/131072.0-((float)dig_T1)/8192.0)*(((float)adc_T)/131072.0-((float)dig_T1)/8192.0))*((float)dig_T3);
	t_fine=((s32)var1)+((s32)var2);
	T=(var1+var2)/5120.0;
	*temp=T;
	
	var1=((float)t_fine/2.0)-64000.0;
	var2=var1*var1*((float)dig_P6)/32768.0;
	var2=var2+var1*((float)dig_P5)*2.0;
	var2=(var2/4.0)+(((float)dig_P4)*65536.0);
	var1=(((float)dig_P3)*var1*var1/524288.0+((float)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((float)dig_P1);
	p=1048576.0-(float)adc_P;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((float)dig_P9)*p*p/2147483648.0;
	var2=p*((float)dig_P8)/32768.0;
	p=p+(var1+var2+((float)dig_P7))/16.0;
	*pres=p;
#endif
}


float getaltitude(float pred)
{
	float p0=101325;
	return(44330*(1-pow(pred/p0,0.19029)));
}

void BMP280_Reg_dump(void)
{
	printf("/****************/\n");
	printf("CONFIG=0x%x\n",MyI2C_ReadByte(0x76,0xF5));
	printf("CTRL_MEAS=0x%x\n",MyI2C_ReadByte(0x76,0xF4));
	printf("STATUS=0x%x\n",MyI2C_ReadByte(0x76,0xF3));
	printf("/****************/\n");
}

