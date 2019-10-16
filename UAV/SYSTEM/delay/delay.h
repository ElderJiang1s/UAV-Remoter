#ifndef __DELAY_H
#define __DELAY_H
#include "m480.h"
#include "cm4.h"

void delay_init(void);
void delay_us(u32 nus);
void delay_ms(u32 nms);
u32 getSysTickCnt(void);
void delay_xms(u32 nms);

#endif
