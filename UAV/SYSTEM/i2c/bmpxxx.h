#ifndef __BMP180_H
#define __BMP180_H
#include "MyI2C.h"
#include "math.h"

//#define BMP180
#ifndef BMP180
#define BMP280
#endif

#ifdef BMP180
#define BMP_I2C_Addr 0x77
#endif
#ifdef BMP280
#define BMP_I2C_Addr 0x76
#endif


void BMP_Calculation(float *temp,float *pres);
u8 bmp_init(void);
float getaltitude(float pred);
void BMP280_Reg_dump(void);

#endif



