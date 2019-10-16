#ifndef _OV7670_H
#define _OV7670_H
#include "cm4.h"
#include "sccb.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//OV7670 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define OV7670_PWDN  	PGout(9)	//POWER DOWN�����ź�
#define OV7670_RST  	PGout(15)//��λ�����ź� 
////////////////////////////////////////////////////////////////////////////////// 
#define OV7670_ID               0X7670  	//OV7670��оƬID
 

#define OV7670_ADDR        		0x42		//OV7670��IIC��ַ
 

 							
u8 OV7670_WR_Reg(u16 reg,u8 data);
u8 OV7670_RD_Reg(u16 reg);
void OV7670_PWDN_Set(u8 sta);
u8 OV7670_Init(void);  
void OV7670_JPEG_Mode(void);
void OV7670_RGB565_Mode(void);
void OV7670_Exposure(u8 exposure);
void OV7670_Light_Mode(u8 mode);
void OV7670_Color_Saturation(u8 sat);
void OV7670_Brightness(u8 bright);
void OV7670_Contrast(u8 contrast);
void OV7670_Sharpness(u8 sharp);
void OV7670_Special_Effects(u8 eft);
void OV7670_Test_Pattern(u8 mode);
void OV7670_Flash_Ctrl(u8 sw);
u8 OV7670_OutSize_Set(u16 offx,u16 offy,u16 width,u16 height);
u8 OV7670_ImageWin_Set(u16 offx,u16 offy,u16 width,u16 height); 
u8 OV7670_Focus_Init(void);
u8 OV7670_Focus_Single(void);
u8 OV7670_Focus_Constant(void);


#endif





















