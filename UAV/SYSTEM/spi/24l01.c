#include "24l01.h"

const u8 TX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const u8 RX_ADDRESS[5]={0x34,0x43,0x10,0x10,0x01};


extern u8 Rx_BUF[32];
extern u8 Tx_BUF[32];

void NRF24L01_Init(void)//24L01
{
	SYS_UnlockReg();
	CLK_SetModuleClock(SPI0_MODULE,CLK_CLKSEL2_SPI0SEL_PLL,0);
	CLK_EnableModuleClock(SPI0_MODULE);
	//����PA1ΪMISO
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA1MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA1MFP_SPI0_MISO;
	//����PA0ΪMOSI
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA0MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA0MFP_SPI0_MOSI;
	//����PA2ΪSCK
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA2MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA2MFP_SPI0_CLK;
	//����PA3ΪCS
	SYS->GPA_MFPL&=~SYS_GPA_MFPL_PA3MFP_Msk;
	SYS->GPA_MFPL|=SYS_GPA_MFPL_PA3MFP_SPI0_SS;
	//����PA4ΪCE
	GPIO_SetMode(PA,BIT4,GPIO_MODE_OUTPUT);
	//����PA5ΪIRQ
	GPIO_SetMode(PA,BIT5,GPIO_MODE_INPUT);
	GPIO_SetPullCtl(PA,BIT5,GPIO_PUSEL_PULL_UP);//����
	SPI_Open(SPI0,SPI_MASTER,SPI_MODE_0,8,9000000);				//CPOL=0	CPHA=0	SPIģʽ0��SCKƵ��9MHz			NRF24L04��֧��ģʽ3
	SPI0->SSCTL&=~SPI_SSCTL_AUTOSS_Msk;											//�Զ��ӻ�Ƭѡ���ܽ�ֹ,Ƭѡ�ź���SPIx_SSCTL[0]����
	//SPI0->CTL|=(SPI_CTL_HALFDPX_Msk|SPI_CTL_DATDIR_Msk);		//��˫���������
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

//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
//	SPI0->CLKDIV=22; //spi�ٶ�Ϊ192/(22+1)Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 STATUS;	
   	SPI_SET_SS_LOW(SPI0);                 //ʹ��SPI����
  	STATUS =SPI_ReadWriteByte(SPI0,reg);//���ͼĴ����� 
  	SPI_ReadWriteByte(SPI0,value);      //д��Ĵ�����ֵ
  	SPI_SET_SS_HIGH(SPI0);                 //��ֹSPI����	   
  	return(STATUS);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	SPI_SET_SS_LOW(SPI0);          //ʹ��SPI����		
  	SPI_ReadWriteByte(SPI0,reg);   //���ͼĴ�����
  	reg_val=SPI_ReadWriteByte(SPI0,0XFF);//��ȡ�Ĵ�������
  	SPI_SET_SS_HIGH(SPI0);          //��ֹSPI����		    
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 STATUS,u8_ctr;	       
  	SPI_SET_SS_LOW(SPI0);           //ʹ��SPI����
  	STATUS=SPI_ReadWriteByte(SPI0,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(SPI0,0XFF);//��������
  	SPI_SET_SS_HIGH(SPI0);       //�ر�SPI����
  	return STATUS;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 STATUS,u8_ctr;	    
 	SPI_SET_SS_LOW(SPI0);          //ʹ��SPI����
  	STATUS = SPI_ReadWriteByte(SPI0,reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_ReadWriteByte(SPI0,*pBuf++); //д������	 
  	SPI_SET_SS_HIGH(SPI0);       //�ر�SPI����
  	return STATUS;          //���ض�����״ֵ̬
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






//����NRF24L01����һ������
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	CE = 0;  //StandBy Iģʽ	
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�  
  CE = 1;	//�ø�CE���������ݷ���
#if Single_Com == 1
	u8 sta;
	
	while(NRF24L01_IRQ!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ;
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
	
#else
	return 0;
#endif
}

void NRF24L01_TxPacket_AP(u8 *txbuf)
{
	CE = 0;  //StandBy Iģʽ	
  NRF24L01_Write_Buf(0xA8,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�  
  CE = 1;	//�ø�CE���������ݷ���
}

//����NRF24L01����һ������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
#if Single_Com == 1
	u8 sta;		    							   
	SPI3_SetSpeed(SPI_BaudRatePrescaler_32); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
			GPIO_SetBits(GPIOE, GPIO_Pin_15 );	
		return 0; 
	}
 	return 1;//û�յ��κ�����
#else
	return 0;
#endif
}					    

//����2401����ģʽ
void NRF24L01_Mode(u8 model)
{
	 SPI_SET_SS_LOW(SPI0);     
   NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
   NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

   NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    00��ʾ��Ӧ��
   NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  Ϊ0��������
   NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10�� ���ط�
   NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,50);       //0x73   ����RFͨ��Ϊ40�����ŵ�����Ϊ2.5GHZ���շ�����һ��
   NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��  

	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x3f); //0f  		 // IRQ�շ�����жϿ���,16λCRC,������0x0f
																													//��λΪ3��ʾ���η����ж�
		SPI_ReadWriteByte(SPI0,0x50);
		SPI_ReadWriteByte(SPI0,0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07);  //06
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + NRF_CONFIG, 0x2e);  //0e 		 // IRQ�շ�����жϿ���,16λCRC,������0x0e
		NRF24L01_Write_Reg(FLUSH_TX,0xff);											//��λbit4MAX_RT bit5 Ϊ1����TX_DS bit6 RX_DS
		NRF24L01_Write_Reg(FLUSH_RX,0xff);											//���ڷɿأ�����ģʽ2e���Ǻ��õ�
		
		SPI_ReadWriteByte(SPI0,0x50);
		SPI_ReadWriteByte(SPI0,0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07); //06
	}
	SPI_SET_SS_HIGH(SPI0); 
}

//����NRF24L01����һ֡����
//u8 tempbuf[64];
u8 NRF24L01_Txframes(u8 *txbuf,u8 packlen)
{
	u8 txresult;
	//��ʼ���ʹ�����֡
	txresult=NRF24L01_TxPacket(txbuf);
  if(packlen>32)
	  txresult=NRF24L01_TxPacket(&txbuf[32]);

  if(packlen>64)
	  txresult=NRF24L01_TxPacket(&txbuf[64]);

  if(packlen>96)txresult=!TX_OK;//���ݳ��ȴ���96������Ϊ�Ǵ���֡

	if(txresult==TX_OK)//�������
	{
		return 1;
	}
	return 0;//����ԭ����ʧ��
}

//u8 NRF24L01_Check_Event(void)
//{
//	u8 sta;
//	sta = NRF24L01_Read_Reg(STATUSS);  //��ȡ״̬�Ĵ�����ֵ;

//	if(sta & RX_OK) //���յ�����
//	{
//		u8 Rx_Len = NRF24L01_Read_Reg(R_RX_PL_WID);
//		if(Rx_Len<33)
//		{
//			NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_BUF,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
//			//��������
//		}else NRF24L01_Write_Reg(FLUSH_RX,0xff);//��ջ�����
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
//	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUSS,sta); //���TX_DS��MAX_RT�жϱ�־
//	if(sta & RX_OK) //���յ�����
//	{
//		return 0;
//	}
//	else
//	{
//		return 1;
//	}
//}















