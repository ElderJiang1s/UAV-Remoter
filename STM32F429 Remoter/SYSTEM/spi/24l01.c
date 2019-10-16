#include "24l01.h"
const u8 TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const u8 RX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01};

SPI_HandleTypeDef SPI1_Handler;  //SPI句柄




u8 SPI_SendByte(u8 data)
{
	u8 Rxdata;
  HAL_SPI_TransmitReceive(&SPI1_Handler,&data,&Rxdata,1, 1000);       
 	return Rxdata;
}


//SPI速度设置函数
//SPI速度=fAPB1/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_2 256
//fAPB1时钟一般为45Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    __HAL_SPI_DISABLE(&SPI1_Handler);            //关闭SPI
    SPI1_Handler.Instance->CR1&=0XFFC7;          //位3-5清零，用来设置波特率
    SPI1_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//设置SPI速度
    __HAL_SPI_ENABLE(&SPI1_Handler);             //使能SPI
}

void NRF24L01_Init(void)//24L01
{
	 GPIO_InitTypeDef GPIO_Initure;
   __HAL_RCC_GPIOG_CLK_ENABLE();         
	SPI1_Handler.Instance=SPI1;                         //SPI1
  SPI1_Handler.Init.Mode=SPI_MODE_MASTER;             //设置SPI工作模式，设置为主模式
  SPI1_Handler.Init.Direction=SPI_DIRECTION_2LINES;   //设置SPI单向或者双向的数据模式:SPI设置为双线模式
  SPI1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       //设置SPI的数据大小:SPI发送接收8位帧结构
  SPI1_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;    //串行同步时钟的空闲状态为高电平
  SPI1_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;         //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
  SPI1_Handler.Init.NSS=SPI_NSS_SOFT;                 //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
  SPI1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
  SPI1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
  SPI1_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        //关闭TI模式
  SPI1_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
  SPI1_Handler.Init.CRCPolynomial=7;                  //CRC值计算的多项式
	HAL_SPI_Init(&SPI1_Handler);//初始化
  __HAL_SPI_ENABLE(&SPI1_Handler);                    //使能SPI1
	SPI1_SetSpeed(SPI_BAUDRATEPRESCALER_8);
	
	
	 
   GPIO_Initure.Pin=GPIO_PIN_8; 
   GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;
   GPIO_Initure.Pull=GPIO_PULLUP;
   GPIO_Initure.Speed=GPIO_SPEED_HIGH; 
   HAL_GPIO_Init(GPIOG,&GPIO_Initure);
	
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,2,0);
	
	CE=0;
	CSN=1;
}



//SPI1底层驱动，时钟使能，引脚配置
//此函数会被HAL_SPI_Init()调用
//hspi:SPI句柄
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef GPIO_Initure;
  __HAL_RCC_GPIOG_CLK_ENABLE();          
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();
	
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6;
  GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
  GPIO_Initure.Pull=GPIO_PULLUP;
  GPIO_Initure.Speed=GPIO_SPEED_HIGH; 
  HAL_GPIO_Init(GPIOG,&GPIO_Initure);//PG8:IRQ	PG7:CS	PG6:CE
	GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;
	GPIO_Initure.Alternate=GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);//PB3:CLK	PB4:MISO	PB5:MOSI
}


//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
//	SPI1->CLKDIV=22; //spi速度为192/(22+1)Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 STATUS;	
   	CSN=0;                //使能SPI传输
  	STATUS =SPI_SendByte(reg);//发送寄存器号 
  	SPI_SendByte(value);      //写入寄存器的值
  	CSN=1;                 //禁止SPI传输	   
  	return(STATUS);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	CSN=0;          //使能SPI传输		
  	SPI_SendByte(reg);   //发送寄存器号
  	reg_val=SPI_SendByte(0XFF);//读取寄存器内容
  	CSN=1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 STATUS,u8_ctr;	       
  	CSN=0;           //使能SPI传输
  	STATUS=SPI_SendByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_SendByte(0XFF);//读出数据
  	CSN=1;       //关闭SPI传输
  	return STATUS;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 STATUS,u8_ctr;	    
 	CSN=0;          //使能SPI传输
  	STATUS = SPI_SendByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_SendByte(*pBuf++); //写入数据	 
  	CSN=1;       //关闭SPI传输
  	return STATUS;          //返回读到的状态值
}				   


void NRF_Reg_dump(void)
{
	printf("/****************/\n");
	printf("NRF_CONFIG=0x%x\n",NRF24L01_Read_Reg(NRF_CONFIG));
	printf("EN_AA=0x%x\n",NRF24L01_Read_Reg(EN_AA));
	printf("EN_RXADDR=0x%x\n",NRF24L01_Read_Reg(EN_RXADDR));
	printf("SETUP_AW=0x%x\n",NRF24L01_Read_Reg(SETUP_AW));
	printf("SETUP_RETR=0x%x\n",NRF24L01_Read_Reg(SETUP_RETR));
	printf("RF_CH=0x%x\n",NRF24L01_Read_Reg(RF_CH));
	printf("RF_SETUP=0x%x\n",NRF24L01_Read_Reg(RF_SETUP));
	printf("STATUSS=0x%x\n",NRF24L01_Read_Reg(STATUSS));
	printf("OBSERVE_TX=0x%x\n",NRF24L01_Read_Reg(OBSERVE_TX));
	printf("CD=0x%x\n",NRF24L01_Read_Reg(CD));
	printf("RX_ADDR_P0=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P0));
	printf("RX_ADDR_P1=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P1));
	printf("RX_ADDR_P2=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P2));
	printf("RX_ADDR_P3=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P3));
	printf("RX_ADDR_P4=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P4));
	printf("RX_ADDR_P5=0x%x\n",NRF24L01_Read_Reg(RX_ADDR_P5));
	printf("TX_ADDR=0x%x\n",NRF24L01_Read_Reg(TX_ADDR));
	printf("RX_PW_P0=0x%x\n",NRF24L01_Read_Reg(RX_PW_P0));
	printf("RX_PW_P1=0x%x\n",NRF24L01_Read_Reg(RX_PW_P1));
	printf("RX_PW_P2=0x%x\n",NRF24L01_Read_Reg(RX_PW_P2));
	printf("RX_PW_P3=0x%x\n",NRF24L01_Read_Reg(RX_PW_P3));
	printf("RX_PW_P4=0x%x\n",NRF24L01_Read_Reg(RX_PW_P4));
	printf("RX_PW_P5=0x%x\n",NRF24L01_Read_Reg(RX_PW_P5));
	printf("NRF_FIFO_STATUS=0x%x\n",NRF24L01_Read_Reg(NRF_FIFO_STATUS));
	printf("/****************/\n");
	
}









//启动NRF24L01发送一次数据
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	CE = 0;  //StandBy I模式	
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节  
  CE = 1;	//置高CE，激发数据发送
	
#if Single_Com == 1
	u8 sta;
	
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值;
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
	
#else
	return 0;
#endif
}

void NRF24L01_TxPacket_AP(u8 *txbuf)
{
	CE = 0;  //StandBy I模式	
  NRF24L01_Write_Buf(0xA8,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节  
  CE = 1;	//置高CE，激发数据发送
}

//启动NRF24L01接收一次数据
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
#if Single_Com == 1
	u8 sta;		    							   
	SPI3_SetSpeed(SPI_BaudRatePrescaler_32); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
			GPIO_SetBits(GPIOE, GPIO_Pin_15 );	
		return 0; 
	}
 	return 1;//没收到任何数据
#else
	return 0;
#endif
}					    

//设置2401工作模式
void NRF24L01_Mode(u8 model)
{
	 CSN = 0;     
   NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
   NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

   NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    00表示不应答
   NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  为0，不接收
   NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 不重发
   NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,50);       //0x73   设置RF通道为40设置信道工作为2.5GHZ，收发必须一致
   NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启  

	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x3f); //0f  		 // IRQ收发完成中断开启,16位CRC,主接收0x0f
																													//高位为3表示屏蔽发送中断
		SPI_SendByte(0x50);
		SPI_SendByte(0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07);  //06
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x2e);  //0e 		 // IRQ收发完成中断开启,16位CRC,主发送0x0e
		NRF24L01_Write_Reg(FLUSH_TX,0xff);											//高位bit4MAX_RT bit5 为1屏蔽TX_DS bit6 RX_DS
		NRF24L01_Write_Reg(FLUSH_RX,0xff);											//对于飞控，发送模式2e才是好用的
		
		SPI_SendByte(0x50);
		SPI_SendByte(0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07); //06
	}
	CSN = 1; 
}

//启动NRF24L01发送一帧数据
//u8 tempbuf[64];
u8 NRF24L01_Txframes(u8 *txbuf,u8 packlen)
{
	u8 txresult;
	//开始发送处理后的帧
	txresult=NRF24L01_TxPacket(txbuf);
  if(packlen>32)
	  txresult=NRF24L01_TxPacket(&txbuf[32]);

  if(packlen>64)
	  txresult=NRF24L01_TxPacket(&txbuf[64]);

  if(packlen>96)txresult=!TX_OK;//数据长度大于96暂且认为是错误帧

	if(txresult==TX_OK)//发送完成
	{
		return 1;
	}
	return 0;//其他原因发送失败
}

//u8 NRF24L01_Check_Event(void)
//{
//	u8 sta;
//	sta = NRF24L01_Read_Reg(STATUSS);  //读取状态寄存器的值;

//	if(sta & RX_OK) //接收到数据
//	{
//		u8 Rx_Len = NRF24L01_Read_Reg(R_RX_PL_WID);
//		if(Rx_Len<33)
//		{
//			NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_BUF,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
//			//处理数据
//		}else NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
//	}
//	if(sta & TX_OK)
//	{
//		
//	}
//	if(sta & MAX_TX)
//	{
//		if(sta & 0x01)	//TX FIFO FULL
//			NRF24L01_Write_Reg(FLUSH_TX,0xff);
//	}
//	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUSS,sta); //清除TX_DS或MAX_RT中断标志
//	if(sta & RX_OK) //接收到数据
//	{
//		return 0;
//	}
//	else
//	{
//		return 1;
//	}
//}













