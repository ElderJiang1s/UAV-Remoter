#include "spi.h"



void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // 双线全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       // 主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   // 8位数据帧
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           // NSS 软件模式
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16 ; // 波特率分频因子，2分频
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  // 高位先行
    SPI_InitStructure.SPI_CRCPolynomial = 7;                            // 使用CRC校验
    //SPI使用模式，CPOL=0，CPHA=0
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         // CPOL=0
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                        // CPHA=0
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;
	GPIO_Init(GPIOA,&GPIO_InitStructure);								//PA4	NSS CSN
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;
	GPIO_Init(GPIOA,&GPIO_InitStructure);								//PA5	SCK
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6;
	GPIO_Init(GPIOA,&GPIO_InitStructure);								//PA6	MISO
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);								//PA7	MOSI
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStructure);								//PB1	CE
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;
	GPIO_Init(GPIOB,&GPIO_InitStructure);								//PB0	IRQ
	
}

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0XFFC7;
	SPI1->CR1|=SPI_BaudRatePrescaler;	//设置SPI2速度 
	SPI_Cmd(SPI1,ENABLE); 
}


u8 SPI_SendByte(u8 data)
{
	// 检查并等待TX缓冲区为空
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
     // 缓冲区为空后向缓冲区写入要发送的字节数据
    SPI_I2S_SendData(SPI1, data);
	// 检查并等待RX缓冲区为非空
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	// 数据发送完毕，从RX缓冲区接收flash返回的数据 
	return SPI_I2S_ReceiveData(SPI1); 
}

