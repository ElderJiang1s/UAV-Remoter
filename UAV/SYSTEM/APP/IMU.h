#ifndef _IMU_H_
#define	_IMU_H_
#include "cm4.h"

/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Get_Radian(s16 *Gyro_in,float *Gyro_out);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* data);
#endif

