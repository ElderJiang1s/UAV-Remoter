#ifndef __MPU6050_H
#define __MPU6050_H
#include "MyI2C.h"

#define mpu6050_addr 0x68

u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU6050_Init(void);
short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
void MPU_Get_Angle(short ax,short ay,short az,float* pitch,float* roll);

#endif


