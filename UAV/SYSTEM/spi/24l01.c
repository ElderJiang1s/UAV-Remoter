#include "24l01.h"

const u8 TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const u8 RX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01};


extern u8 Rx_BUF[32];
extern u8 Tx_BUF[32];

void NRF24L01_Init(void)//24L01
{
	SYS_UnlockReg();
	CLK_SetModuleClock(SPI0_MODULE,CLK_CLKSEL2_SPI0SEL_PLL,0);
	CLK_EnableModuleClock(SPI0_MODULE);
	//设置PA1为MISO
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA1MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA1MFP_SPI0_MISO;
	//设置PA0为MOSI
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA0MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA0MFP_SPI0_MOSI;
	//设置PA2为SCK
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA2MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA2MFP_SPI0_CLK;
	//设置PA3为CS
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA3MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA3MFP_SPI0_SS;
	//设置PA4为CE
	GPIO_SetMode(PA,BIT4,GPIO_MODE_OUTPUT);
	//设置PA5为IRQ
	GPIO_SetMode(PA,BIT5,GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PA,BIT5,GPIO_PUSEL_PULL_UP);//上拉
	SPI_Open(SPI0,SPI_MASTER,SPI_MODE_0,8,9000000);				//CPOL=0	CPHA=0	SPI模式0，SCK频率9MHz			NRF24L04不支持模式3
	SPI0->SSCTL&=~SPI_SSCTL_AUTOSS_Msk;											//自动从机片选功能禁止,片选信号由SPIx_SSCTL[0]控制
	//SPI0->CTL|=(SPI_CTL_HALFDPX_Msk|SPI_CTL_DATDIR_Msk);		//半双工数据输出
	SYS_LockReg();
	CE=0;
	SPI_SET_SS_HIGH(SPI0);
	if(NRF24L01_Check())
	{
		while(1);
	}
}

u8 SPI_ReadWriteByte(SPI_T* SPIx,u8 Data)
{		
	while(SPIx->STATUS&SPI_STATUS_BUSY_Msk);
	SPI0->TX=Data;
	while(SPIx->STATUS&SPI_STATUS_BUSY_Msk);		
	return(SPI0->RX);
}

//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
//	SPI0->CLKDIV=22; //spi速度为192/(22+1)Mhz（24L01的最大SPI时钟为10Mhz）   	 
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
   	SPI_SET_SS_LOW(SPI0);                 //使能SPI传输
  	STATUS =SPI_ReadWriteByte(SPI0,reg);//发送寄存器号 
  	SPI_ReadWriteByte(SPI0,value);      //写入寄存器的值
  	SPI_SET_SS_HIGH(SPI0);                 //禁止SPI传输	   
  	return(STATUS);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	SPI_SET_SS_LOW(SPI0);          //使能SPI传输		
  	SPI_ReadWriteByte(SPI0,reg);   //发送寄存器号
  	reg_val=SPI_ReadWriteByte(SPI0,0XFF);//读取寄存器内容
  	SPI_SET_SS_HIGH(SPI0);          //禁止SPI传输		    
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
  	SPI_SET_SS_LOW(SPI0);           //使能SPI传输
  	STATUS=SPI_ReadWriteByte(SPI0,reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(SPI0,0XFF);//读出数据
  	SPI_SET_SS_HIGH(SPI0);       //关闭SPI传输
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
 	SPI_SET_SS_LOW(SPI0);          //使能SPI传输
  	STATUS = SPI_ReadWriteByte(SPI0,reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_ReadWriteByte(SPI0,*pBuf++); //写入数据	 
  	SPI_SET_SS_HIGH(SPI0);       //关闭SPI传输
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
	 SPI_SET_SS_LOW(SPI0);     
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
		SPI_ReadWriteByte(SPI0,0x50);
		SPI_ReadWriteByte(SPI0,0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07);  //06
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x2e);  //0e 		 // IRQ收发完成中断开启,16位CRC,主发送0x0e
		NRF24L01_Write_Reg(FLUSH_TX,0xff);											//高位bit4MAX_RT bit5 为1屏蔽TX_DS bit6 RX_DS
		NRF24L01_Write_Reg(FLUSH_RX,0xff);											//对于飞控，发送模式2e才是好用的
		
		SPI_ReadWriteByte(SPI0,0x50);
		SPI_ReadWriteByte(SPI0,0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07); //06
	}
	SPI_SET_SS_HIGH(SPI0); 
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















