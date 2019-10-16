#include "usart.h"
#include "delay.h"
/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2015/9/7
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.0修改说明 
////////////////////////////////////////////////////////////////////////////////// 	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
////接收状态
////bit15，	接收完成标志
////bit14，	接收到0x0d
////bit13~0，	接收到的有效字节数目
//u16 USART_RX_STA=0;       //接收状态标记	

//u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound)
{	
	//UART 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
	__HAL_UART_ENABLE_IT(&UART1_Handler, UART_IT_RXNE);

}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//高速
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//复用为USART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,1,0);			//抢占优先级3，子优先级3
#endif	
	}

}

extern QueueHandle_t UARTRx_Queue;//UART接收到的数据队列
void USART1_IRQHandler(void)                	
{
	BaseType_t tmp;
	static u8 uart_rxdata[32];
	static u8 aa=0,af=0,fun=0,len=0,cnt=0,sum=0;
	u8 dat;
	dat=USART1->DR&0xFF;
	if(dat==0xAA&&aa==0)
	{
		aa=1;
		uart_rxdata[0]=dat;
	}else if(dat==0xAF&&aa==1&&af==0 )
	{
		af=1;
		uart_rxdata[1]=dat;
	}else if(aa==1&&af==1&&fun==0)
	{
		fun=dat;
		uart_rxdata[2]=fun;
	}else if(aa==1&&af==1&&fun!=0&&len==0)
	{
		len=dat;
		uart_rxdata[3]=len;
	}else if(aa==1&&af==1&&fun!=0&&len!=0&&cnt!=len)
	{
		uart_rxdata[4+cnt]=dat;
		cnt++;
	}else if(aa==1&&af==1&&fun!=0&&len!=0&&cnt==len)
	{
		sum=dat;
		uart_rxdata[4+cnt]=sum;
		xQueueSendFromISR(UARTRx_Queue,uart_rxdata,&tmp);//发送UART接收到的数据到队列
		if(tmp == pdTRUE)
		{
			portYIELD_FROM_ISR(tmp);
		}
		aa=0;
		af=0;
		fun=0;
		len=0;
		cnt=0;
		sum=0;
	}
	HAL_UART_IRQHandler(&UART1_Handler);
}


void usart_dma_init()
{
}








#endif

