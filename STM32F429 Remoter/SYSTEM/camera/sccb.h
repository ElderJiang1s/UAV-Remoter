#ifndef __SCCB_H
#define __SCCB_H
#include "cm4.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//OVϵ������ͷ SCCB ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									;'

////////////////////////////////////////////////////////////////////////////////// 	
//IO��������
#define SCCB_SDA_IN()  {GPIOD->MODER&=0xFFFF3FFF;}	//PD7����ģʽ
#define SCCB_SDA_OUT() {GPIOD->MODER&=0xFFFF3FFF;GPIOD->MODER|=0x4000;}  //PD7���ģʽ
//IO����
#define SCCB_SCL        PDout(6)    //SCL
#define SCCB_SDA        PDout(7)    //SDA

#define SCCB_READ_SDA   PDin(7)     //����SDA 


///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
u8 SCCB_WR_Byte(u8 dat);
u8 SCCB_RD_Byte(void);
#endif

