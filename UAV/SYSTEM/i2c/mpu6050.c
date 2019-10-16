#include "mpu6050.h"
#include "math.h"






u8 MPU_Set_Rate(u16 rate)//设置采样率
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MyI2C_WriteByte(0x68,0x19,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MyI2C_WriteByte(0x68,0x1A,data);//设置数字低通滤波器  
}

u8 MPU6050_Init(void)
{
//	delay_ms(1000);
//	MyI2C_Init();
	u8 res;
	MyI2C_WriteByte(0x68,0x6B,0x80);//电源管理寄存器1//复位MPU6050
	delay_ms(100);
	MyI2C_WriteByte(0x68,0x6B,0x00);
	MyI2C_WriteByte(0x68,0x1B,3<<3);//陀螺仪配置寄存器//陀螺仪传感器,±2000dps
	MyI2C_WriteByte(0x68,0x1C,0<<3);//加速度计配置寄存器//加速度传感器,±2g
	MPU_Set_Rate(50);//设置采样率50Hz	
	MyI2C_WriteByte(0x68,0x38,0X00);	//关闭所有中断
	MyI2C_WriteByte(0x68,0x6A,0X00);	//I2C主模式关闭
	MyI2C_WriteByte(0x68,0x23,0X00);	//关闭FIFO
	MyI2C_WriteByte(0x68,0x37,0X80);	//INT引脚低电平有效
	res=MyI2C_ReadByte(0x68,0x75);//器件ID寄存器
	if(res==0x68)//器件ID正确
	{
		MyI2C_WriteByte(0x68,0x6B,0X01);	//设置CLKSEL,PLL X轴为参考
		MyI2C_WriteByte(0x68,0x6C,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(1000);						//设置采样率
 	}
	else
	{
		while(1);
	}
	return 0;
	
}


//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	I2C_Read_Len(0x68,0x41,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
//gx=2000*gyr_x/32768
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=I2C_Read_Len(0x68,0x43,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
//ax=2g*acc_x/32768
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=I2C_Read_Len(0x68,0x3B,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

void MPU_Get_Angle(short ax,short ay,short az,float* pitch,float* roll)
{
	double sqrt1,sqrt2;
	sqrt1=sqrt((double)(ay*ay)+(double)(az*az))/16384.0f;
	sqrt2=sqrt((double)(ax*ax)+(double)(az*az))/16384.0f;
	if(sqrt1>1)sqrt1=1;
	if(sqrt2>1)sqrt2=1;
	*pitch=(float)acos(sqrt1)/0.01745f;
	*roll=(float)acos(sqrt2)/0.01745f;
	if(ay>0)*roll=-*roll;
	if(ax<0)*pitch=-*pitch;
}



