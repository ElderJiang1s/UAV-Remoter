/************************************************************************************
*  Copyright (c), 2014, HelTec Automatic Technology co.,LTD.
*            All rights reserved.
*
* Http:    www.heltec.cn
* Email:   cn.heltec@gmail.com
* WebShop: heltec.taobao.com
*
* File name: OLED_I2C.c
* Project  : HelTec.uvprij
* Processor: STM32F103C8T6
* Compiler : MDK fo ARM
* 
* Author : 小林
* Version: 1.00
* Date   : 2014.4.8
* Email  : hello14blog@gmail.com
* Modification: none
* 
* Description:128*64点阵的OLED显示屏驱动文件，仅适用于惠特自动化(heltec.taobao.com)的SD1306驱动IIC通信方式显示屏
*
* Others: none;
*
* Function List:
*	1. void I2C_Configuration(void) -- 配置CPU的硬件I2C
* 2. void MyI2C_WriteByte_HARD(uint8_t addr,uint8_t data) -- 向寄存器地址写一个byte的数据
* 3. void WriteCmd(unsigned char I2C_Command) -- 写命令
* 4. void WriteDat(unsigned char I2C_Data) -- 写数据
* 5. void OLED_Init(void) -- OLED屏初始化
* 6. void OLED_SetPos(unsigned char x, unsigned char y) -- 设置起始点坐标
* 7. void OLED_Fill(unsigned char fill_Data) -- 全屏填充
* 8. void OLED_CLS(void) -- 清屏
* 9. void OLED_ON(void) -- 唤醒
* 10. void OLED_OFF(void) -- 睡眠
* 11. void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- 显示字符串(字体大小有6*8和8*16两种)
* 12. void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N) -- 显示中文(中文需要先取模，然后放到codetab.h中)
* 13. void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]) -- BMP图片
*
* History: none;
*
*************************************************************************************/

#include "OLED_I2C.h"
#include "codetab.h"
#include "string.h"
#include "math.h"
extern _ARMABI_PURE int abs(int /*x*/);
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 
u8 OLED_GRAM[128][8];

void sss(u32 t)
{
	u32 temp=11000*t;
	while(temp){temp--;}
}


void WriteCmd(unsigned char I2C_Command)//写命令
{
	MyI2C_WriteByte(OLED_ADDRESS>>1,0x00,I2C_Command);
}

void WriteDat(unsigned char I2C_Data)//写数据
{
	MyI2C_WriteByte(OLED_ADDRESS>>1,0x40,I2C_Data);
}

//更新显存到LCD		 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		WriteCmd(0xb0+i);    //设置页地址（0~7）
		WriteCmd(0x10);      //设置显示位置—列低地址
		WriteCmd(0x01);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)WriteDat(OLED_GRAM[n][i]); 
	} 
	//memset(OLED_GRAM,0x0,1024);	//清空显存
}

//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	y=63-y;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//画线
void OLED_Line(int x0,int y0,int x1,int y1)
{
	int dx,dy,n,k;
	float xinc,yinc,x,y;
	dx=x1-x0;dy=y1-y0;
	if(abs(dx)>abs(dy))
		{n=abs(dx);}
	else
		{n=abs(dy);}
	xinc=(float)dx/n;
	yinc=(float)dy/n;
	x=(float)x0;
	y=(float)y0;
	for(k=1;k<=n;k++)
	{
		OLED_DrawPoint((int)(x+0.5f),(int)(y+0.5f),1);
		x+=xinc;
		y+=yinc;
	}
	OLED_Refresh_Gram();
}


void OLED_Init(void)
{
	MyI2C_Init();
	sss(100); //这里的延时很重要	内部延时 没用SysTick
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
	OLED_CLS();
}

void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}

void OLED_Fill(unsigned char fill_Data)//全屏填充
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}

void OLED_CLS(void)//清屏
{
	OLED_Fill(0x00);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
// Description    : 显示codetab.h中的ASCII字符,有6*8和8*16可选择
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); N:汉字在codetab.h中的索引
// Description    : 显示codetab.h中的汉字,16*16点阵
//--------------------------------------------------------------
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	u8 t,adder=0;
	OLED_SetPos(x,y);	
    for(t=0;t<16;t++)
	{
		WriteDat(F16x16[2*N][t]);
		adder+=1;
    }	
	OLED_SetPos(x,y+1);	
    for(t=0;t<16;t++)
	{			
		WriteDat(F16x16[2*N+1][t]);
		adder+=1;
	}					
}

//--------------------------------------------------------------
// Prototype      : void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- 起始点坐标(x0:0~127, y0:0~7); x1,y1 -- 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
// Description    : 显示江泽民
//--------------------------------------------------------------
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1)
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP1[j++]);
		}
	}
}

void OLED_Printf(u8 line,char* fomat, ...)
{
	u8 c1,c2;
	char temp[30];
	u8 i=0;
	u32 addr,ss;
	u32 base_addr=(u32)&fomat;
	addr=(u32)fomat;
	while((addr-(u32)fomat)<29)
	{
		addr++;
		if(i==2)break;
		ss=(u32)strchr((const char*)addr,0x25);
		if(ss!=0)
		{
			i++;
			addr=ss;
			switch(i)
			{
				case 1:
					if(*((u8*)ss+1)==0x2E)
					{
						c1=0x66;
					}
					else
					{
						c1=*((u8*)ss+1);
					}
				break;
				case 2:
					if(*((u8*)ss+1)==0x2E)
					{
						c2=0x66;
					}
					else
					{
						c2=*((u8*)ss+1);
					}
				break;
				default:strcpy(temp,"print error");
			}
			
			
			
		}
	}
	switch (i)
	{
		case 0:sprintf(temp,"%s",fomat);break;
		case 1:
			switch(c1)
			{
				case 0x63:sprintf(temp,fomat,*((char*)(base_addr+4)));break;//c
				case 0x64:sprintf(temp,fomat,*((int*)(base_addr+4)));break;//d
				case 0x66:sprintf(temp,fomat,*((double*)(base_addr+4)));;break;//f
				case 0x75:sprintf(temp,fomat,*((u32*)(base_addr+4)));break;//u
				case 0x78:sprintf(temp,fomat,*((u32*)(base_addr+4)));break;//x
			}
		break;
		case 2:
			switch(c1)
			{
				case 0x63:
					switch(c2)
					{
						case 0x63:sprintf(temp,fomat,*((char*)(base_addr+4)),*((char*)(base_addr+8)));break;//c
						case 0x64:sprintf(temp,fomat,*((char*)(base_addr+4)),*((int*)(base_addr+8)));break;//d
						case 0x66:sprintf(temp,fomat,*((char*)(base_addr+4)),*((double*)(base_addr+12)));break;//f
						case 0x75:sprintf(temp,fomat,*((char*)(base_addr+4)),*((u32*)(base_addr+8)));break;//u
						case 0x78:sprintf(temp,fomat,*((char*)(base_addr+4)),*((u32*)(base_addr+8)));break;//x
					}
				break;
				case 0x64:
					switch(c2)
					{
						case 0x63:sprintf(temp,fomat,*((int*)(base_addr+4)),*((char*)(base_addr+8)));break;//c
						case 0x64:sprintf(temp,fomat,*((int*)(base_addr+4)),*((int*)(base_addr+8)));break;//d
						case 0x66:sprintf(temp,fomat,*((int*)(base_addr+4)),*((double*)(base_addr+12)));break;//f
						case 0x75:sprintf(temp,fomat,*((int*)(base_addr+4)),*((u32*)(base_addr+8)));break;//u
						case 0x78:sprintf(temp,fomat,*((int*)(base_addr+4)),*((u32*)(base_addr+8)));break;//x
					}
				break;
				case 0x66:
					switch(c2)
					{
						case 0x63:sprintf(temp,fomat,*((double*)(base_addr+4)),*((char*)(base_addr+8)));break;//c
						case 0x64:sprintf(temp,fomat,*((double*)(base_addr+4)),*((int*)(base_addr+8)));break;//d
						case 0x66:sprintf(temp,fomat,*((double*)(base_addr+4)),*((double*)(base_addr+12)));break;//f
						case 0x75:sprintf(temp,fomat,*((double*)(base_addr+4)),*((u32*)(base_addr+8)));break;//u
						case 0x78:sprintf(temp,fomat,*((double*)(base_addr+4)),*((u32*)(base_addr+8)));break;//x
					}
				break;
				case 0x75:
					switch(c2)
					{
						case 0x63:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((char*)(base_addr+8)));break;//c
						case 0x64:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((int*)(base_addr+8)));break;//d
						case 0x66:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((double*)(base_addr+12)));break;//f
						case 0x75:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((u32*)(base_addr+8)));break;//u
						case 0x78:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((u32*)(base_addr+8)));break;//x
					}
					case 0x78:
					switch(c2)
					{
						case 0x63:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((char*)(base_addr+8)));break;//c
						case 0x64:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((int*)(base_addr+8)));break;//d
						case 0x66:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((double*)(base_addr+12)));break;//f
						case 0x75:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((u32*)(base_addr+8)));break;//u
						case 0x78:sprintf(temp,fomat,*((u32*)(base_addr+4)),*((u32*)(base_addr+8)));break;//x
					}
				break;
			}
		break;
		default:strcpy(temp,"print error");
	}
	OLED_ShowStr(0,line,(char*)temp,2);
}

void OLED_Line_CLR(u8 line)
{
	OLED_ShowStr(0,line,"               ",2);
}


