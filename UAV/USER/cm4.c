#include "cm4.h"
//2019年3月1日23:15:01 		创建
//2019年3月9日17:16:22 		添加TIMER	ADC
//2019年3月23日23:54:56		添加DMA
//2019年5月11日11:00:08		添加USART


/**********************************************************/
/*            						CLK						                  */
/**********************************************************/
void Set_HCLK_192Mhz(void)										//设置HCLK为192MHz
{
	SYS_UnlockReg();
	CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	CLK_SetCoreClock(FREQ_192MHZ);
	SystemCoreClockUpdate();
	SYS_LockReg();
}

/**********************************************************/
/*            						GPIO					                  */
/**********************************************************/
void LED_Init(void)			//led1:PB15	led2:PB14
{
	GPIO_SetMode(PB,BIT14,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PB,BIT15,GPIO_MODE_OUTPUT);
	led1=0;
	led2=0;
}

void KEY_Init(void)		//key1:PC14				GPC_IRQHandler
{
	//NVIC_SetPriorityGrouping(2);						//设置NVIC优先级分组
	GPIO_SetMode(PC,BIT14,GPIO_MODE_INPUT);		//设置PC14为输入
	GPIO_EnableInt(PC,14,GPIO_INT_FALLING);		//打开FALLING中断
	NVIC_EnableIRQ(GPC_IRQn);									//打开NVIC的GPIOC中断
	NVIC_SetPriority(GPC_IRQn,1);							//设置GPIOC中断优先级
	GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK,GPIO_DBCTL_DBCLKSEL_32768);	//打开消抖时钟
	GPIO_ENABLE_DEBOUNCE(PC,BIT14);						//打开PC14消抖功能
}



/**********************************************************/
/*            						TIMER					                  */
/**********************************************************/

void TIM0_IT_Init(u32 nus)//TMR0_IRQHandler()		//TIMERx->INTSTS|=TIMER_INTSTS_TIF_Msk;
{
	SYS_UnlockReg();
	CLK_SetModuleClock(TMR0_MODULE,CLK_CLKSEL1_TMR0SEL_HXT,0);
	CLK_EnableModuleClock(TMR0_MODULE);
	TIMER_Open(TIMER0,TIMER_PERIODIC_MODE,1000000/nus);
	TIMER0->CTL|=TIMER_CTL_INTEN_Msk;								//ENABLE中断
	NVIC_EnableIRQ(TMR0_IRQn);											//打开NVIC的TIMER0中断
	NVIC_SetPriority(TMR0_IRQn,2);									//设置TIMER0中断优先级
	TIMER_Start(TIMER0);
	SYS_LockReg();
}

void EPWM0_Init(void)
{
	SYS_UnlockReg();
	CLK_SetModuleClock(EPWM0_MODULE,CLK_CLKSEL2_EPWM0SEL_PLL,0);
	CLK_EnableModuleClock(EPWM0_MODULE);
	SYS->GPB_MFPL&=~SYS_GPB_MFPL_PB4MFP_Msk;
	SYS->GPB_MFPL|=SYS_GPB_MFPL_PB4MFP_EPWM0_CH1;
	SYS->GPB_MFPL&=~SYS_GPB_MFPL_PB3MFP_Msk;
	SYS->GPB_MFPL|=SYS_GPB_MFPL_PB3MFP_EPWM0_CH2;
	SYS->GPB_MFPL&=~SYS_GPB_MFPL_PB2MFP_Msk;
	SYS->GPB_MFPL|=SYS_GPB_MFPL_PB2MFP_EPWM0_CH3;
	SYS->GPB_MFPL&=~SYS_GPB_MFPL_PB1MFP_Msk;
	SYS->GPB_MFPL|=SYS_GPB_MFPL_PB1MFP_EPWM0_CH4;
	EPWM_ConfigOutputChannel(EPWM0,1 , 50000, 0);
	EPWM_ConfigOutputChannel(EPWM0,2 , 50000, 0);
	EPWM_ConfigOutputChannel(EPWM0,3 , 50000, 0);
	EPWM_ConfigOutputChannel(EPWM0,4 , 50000, 0);
//	EPWM1->PERIOD[0]=0xFF;
//	EPWM1->PERIOD[1]=0xFF;
//	EPWM1->CMPDAT[0]=0x7F;
//	EPWM1->CMPDAT[1]=0x7F;
//	EPWM1->CLKPSC[0]=1;							//375.9KHz
	
	EPWM_EnableOutput(EPWM0,EPWM_CH_1_MASK|EPWM_CH_2_MASK|EPWM_CH_3_MASK|EPWM_CH_4_MASK);
	EPWM_Start(EPWM0, EPWM_CH_1_MASK|EPWM_CH_2_MASK|EPWM_CH_3_MASK|EPWM_CH_4_MASK);
	SYS_LockReg();
}

void EPWM_SetDutyCycle(EPWM_T *epwm,u32 duty,u32 nch)
{
	epwm->CMPDAT[nch]=(u32)(epwm->PERIOD[nch]*(float)(duty/100.0f));
}



/**********************************************************/
/*            						ADC						                  */
/**********************************************************/
void ADC_Init(void)						//EADC_CH8:PB8
{
	SYS_UnlockReg();
	SYS->VREFCTL&=~(SYS_VREFCTL_PRELOAD_SEL_Msk|SYS_VREFCTL_VREFCTL_Msk);
	SYS->VREFCTL|=1<<SYS_VREFCTL_PRELOAD_SEL_Pos;						//VREF预加载时间310us 1uF电容
	SYS->VREFCTL|=0<<SYS_VREFCTL_VREFCTL_Pos;									//VREF由外接电压决定
	CLK_SetModuleClock(EADC_MODULE,0,CLK_CLKDIV0_EADC(1));
	CLK_EnableModuleClock(EADC_MODULE);
	SYS->GPB_MFPH&=~SYS_GPB_MFPH_PB8MFP_Msk;
	SYS->GPB_MFPH|=SYS_GPB_MFPH_PB8MFP_EADC0_CH8;
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT8);								//关闭数字输入通道
	EADC_CONV_RESET(EADC);																	//ADC复位
	while(EADC->CTL&EADC_CTL_ADCRST_Msk);										//等待复位完成
	EADC_Open(EADC,EADC_CTL_DIFFEN_SINGLE_END);
	EADC->CALCTL|=EADC_CALCTL_CALSEL_Msk;										//启动校准
	while(!(EADC->CALCTL&EADC_CALCTL_CALDONE_Msk));					//等待校准完成
	SYS_LockReg();
}


u32 ADC_GetData(u32 nch)
{
	EADC_ConfigSampleModule(EADC,0,EADC_SOFTWARE_TRIGGER,nch);			//ADC模块0转换通道nch
	EADC->SCTL[0]&=~EADC_SCTL_EXTSMPT_Msk;
	EADC->SCTL[0]|=0xFF<<EADC_SCTL_EXTSMPT_Pos;											//ADC采样时间加长0xFF
	EADC_START_CONV(EADC,BIT0);																			//开始模块0转换
	while(!(EADC->DAT[0]&EADC_DAT_VALID_Msk));											//等待转换完成
	return(EADC->DAT[0]&0xFFFF);
}

float ADC_GetTemp(void)
{
	float temp,offset;
	if((SYS->IVSCTL&SYS_IVSCTL_VTEMPEN_Msk)==0)
	{
		SYS->IVSCTL|=SYS_IVSCTL_VTEMPEN_Msk;
	}
	EADC_START_CONV(EADC,BIT17);												//开始模块17转换
	while(!(EADC->DAT[17]&EADC_DAT_VALID_Msk));					//等待转换完成
	temp=(EADC->DAT[17]&0xFFFF)/4096.0f*3300.0f;
	offset=(temp-713)/(0-1.82);													//0°C:713mV		-1.82mC/°C
	return(offset);
}


/**********************************************************/
/*            						DAC						                  */
/**********************************************************/
void DAC_Init(void)												//PB12:DAC0_OUT		PB13:DAC1_OUT
{
	SYS_UnlockReg();
	CLK_EnableModuleClock(DAC_MODULE);
	SYS->IPRST2|=SYS_IPRST2_DACRST_Msk;
	SYS->IPRST2&=~SYS_IPRST2_DACRST_Msk;
	SYS->GPB_MFPH&=~SYS_GPB_MFPH_PB12MFP_Msk;
	SYS->GPB_MFPH&=~SYS_GPB_MFPH_PB13MFP_Msk;
	SYS->GPB_MFPH|=SYS_GPB_MFPH_PB12MFP_DAC0_OUT;
	SYS->GPB_MFPH|=SYS_GPB_MFPH_PB13MFP_DAC1_OUT;
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT12|BIT13);			//关闭数字输入通道
	DAC_Open(DAC0,0,DAC_WRITE_DAT_TRIGGER);
	DAC_Open(DAC1,0,DAC_WRITE_DAT_TRIGGER);
	SYS_LockReg();
}

void DAC_Output(DAC_T *dac,u32 val)
{
	while(DAC_IS_BUSY(dac,0));											//等待DAC ready
	DAC_WRITE_DATA(dac,0,val);
	while(DAC_GET_INT_FLAG(dac, 0)==0);							//等待DAC转换finish	
}






/**********************************************************/
/*            						RTC								              */
/**********************************************************/

void LXT_Init(void)
{
	SYS_UnlockReg();
//	CLK_EnableModuleClock(RTC_MODULE);
	SYS->GPF_MFPL&=~SYS_GPF_MFPL_PF4MFP_Msk;
	SYS->GPF_MFPL&=~SYS_GPF_MFPL_PF5MFP_Msk;
	SYS->GPF_MFPL|=SYS_GPF_MFPL_PF4MFP_X32_OUT;
	SYS->GPF_MFPL|=SYS_GPF_MFPL_PF5MFP_X32_IN;
	CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
	if(CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk)==0)
	{
		printf("lxt is not stable\n");
	}
	SYS_LockReg();
}



/**********************************************************/
/*            						DMA								              */
/**********************************************************/
//				通道	数据宽度	传输数量	所选外设			源地址		目的地						源地址是否增加	目的地地址是否增加
//DMA_Init(0,PDMA_WIDTH_8,44,				PDMA_UART0_TX,(u32)data,(u32)&UART0->DAT,PDMA_SAR_INC,		PDMA_DAR_FIX);
void DMA_Init(u32 nch,u32 Width,u32 TransCount,u32 Peripheral,u32 SrcAddr,u32 DirAddr,u32 SrcCtrl,u32 DstCtrl)
{
	
	SYS_UnlockReg();
	CLK_EnableModuleClock(PDMA_MODULE);
	PDMA_Open(PDMA,1<<nch);
	PDMA_SetTransferCnt(PDMA,nch, Width, TransCount);
	PDMA_SetTransferAddr(PDMA,nch, SrcAddr,SrcCtrl, DirAddr, DstCtrl);
	PDMA_SetTransferMode(PDMA,nch, Peripheral, 0, 0);
	PDMA_SetBurstType(PDMA, nch, PDMA_REQ_SINGLE, PDMA_BURST_128);
//	PDMA_EnableInt(PDMA,nch, PDMA_INT_TRANS_DONE);
//	NVIC_EnableIRQ(PDMA_IRQn);
//	NVIC_SetPriority(PDMA_IRQn,1);
	SYS_LockReg();
	
}


/**********************************************************/
/*            					USART						                  */
/**********************************************************/
void uart_init(u32 bound)//PB12:UART0_RX		PB13:UART0_TX
{
	CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HXT,CLK_CLKDIV0_UART0(1));
	CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
	SYS->GPB_MFPH&=~(SYS_GPB_MFPH_PB12MFP_UART0_RXD|SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	SYS->GPB_MFPH|=SYS_GPB_MFPH_PB12MFP_UART0_RXD|SYS_GPB_MFPH_PB13MFP_UART0_TXD;
	UART_SetLineConfig(UART0, bound, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
	UART0->FIFO&=~UART_FIFO_RXOFF_Msk;
//	UART_EnableInt(UART0,UART_INTEN_RDAIEN_Msk);
//	NVIC_SetPriority(UART0_IRQn,1);
//	NVIC_EnableIRQ(UART0_IRQn);
}


float mapf(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
	return(val/(I_Max-I_Min)*(O_Max-O_Min) + O_Min);
}


/**********************************************************/
/*            					OTHER						                  */
/**********************************************************/


//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
