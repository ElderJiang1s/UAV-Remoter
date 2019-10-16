#include "cm4.h"
/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
//2019年3月1日23:15:01 		创建
//2019年3月9日17:16:22 		添加TIMER	ADC
//2019年3月23日23:54:56		添加DMA
//2019年6月5日23:05:54		修改至M485KIDAE
//2019年7月23日00:45:24		修改至STM32F429ZGT6
//2019年10月1日21:48:43		添加F429 HAL库DMA

/**********************************************************/
/*            						GPIO					                  */
/**********************************************************/
void LED_Init(void)//LED1:PF8		LED2:PF9		LED3:PF10
{
	 GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOF_CLK_ENABLE();          
	
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull=GPIO_PULLDOWN;
    GPIO_Initure.Speed=GPIO_SPEED_HIGH; 
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_8,GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_RESET);	
}

void KEY_Init(void)		//KEY1:PE2  KEY2:PE3  KEY3:PE4  
{
	 GPIO_InitTypeDef GPIO_Initure;
   __HAL_RCC_GPIOE_CLK_ENABLE();          
   GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4; 
   GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;
   GPIO_Initure.Pull=GPIO_PULLUP;
   GPIO_Initure.Speed=GPIO_SPEED_HIGH; 
   HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	HAL_NVIC_SetPriority(EXTI2_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);   
	
	HAL_NVIC_SetPriority(EXTI3_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);   
	
	HAL_NVIC_SetPriority(EXTI4_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);   
}

//中断服务函数
void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
/**********************************************************/
/*            						SRAM					                  */
/**********************************************************/
SRAM_HandleTypeDef sram;
void SRAM_Init(void)
{
	FMC_NORSRAM_TimingTypeDef Timing;
	__HAL_RCC_FMC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	
	sram.Instance = FMC_NORSRAM_DEVICE;
  sram.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* sram.Init */
  sram.Init.NSBank = FMC_NORSRAM_BANK3;
  sram.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  sram.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  sram.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  sram.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  sram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  sram.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  sram.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  sram.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  sram.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  sram.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  sram.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  sram.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  sram.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  sram.Init.PageSize = FMC_PAGE_SIZE_NONE;
	
	
	Timing.AddressSetupTime = 0;//地址建立时间
  Timing.AddressHoldTime = 0;//地址保持时间
  Timing.DataSetupTime = 8;//数据保持时间
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 0;
  Timing.DataLatency = 0;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
	HAL_SRAM_Init(&sram,&Timing,&Timing);
}


void  HAL_SRAM_MspInit(SRAM_HandleTypeDef *sram)
{
	
	GPIO_InitTypeDef GPIO_Initure;
	
	
	GPIO_Initure.Pin= (3<<0)|(3<<4)|(0XFF<<8);
  GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用   GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
  GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
  GPIO_Initure.Alternate=GPIO_AF12_FMC;       //复用为FMC    
  HAL_GPIO_Init(GPIOD,&GPIO_Initure);          //初始化PD0,1,4,5,8~15
	
  GPIO_Initure.Pin = (3<<0)|(0X1FF<<7);//PE0,1,7~15,AF OUT
  HAL_GPIO_Init(GPIOE,&GPIO_Initure); 
 	GPIO_Initure.Pin = (0X3F<<0)|(0XF<<12); 	//PF0~5,12~15
  HAL_GPIO_Init(GPIOF,&GPIO_Initure);
	GPIO_Initure.Pin = (0X3F<<0)|GPIO_PIN_10;//PG0~5,10
  HAL_GPIO_Init(GPIOG,&GPIO_Initure);
	
}

/**********************************************************/
/*            						TIMER					                  */
/**********************************************************/
TIM_HandleTypeDef TIM6_Handler;
void TIM6_IT_Init(u32 arr)
{
	TIM6_Handler.Instance=TIM6;                         //通用定时器6
  TIM6_Handler.Init.Prescaler=9000-1;                     //分频系数
  TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
  TIM6_Handler.Init.Period=arr;                        //自动装载值
  TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
  HAL_TIM_Base_Init(&TIM6_Handler);
  HAL_TIM_Base_Start_IT(&TIM6_Handler); //使能定时器6和定时器6更新中断：TIM_IT_UPDATE  
}
//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();            //使能TIM6时钟
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn,0,1);    //设置中断优先级，抢占优先级0，子优先级1
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //开启ITM6中断   
	}
}

void TIM6_DAC_IRQHandler ()
{
	HAL_TIM_IRQHandler(&TIM6_Handler);
}
extern u8 ov_frame;
//回调函数，定时器中断服务函数调用
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	u8* tt[10];
//    if(htim==(&TIM6_Handler))
//    {
//        LED1=!LED1;        //LED1反转
//			printf("fps=%d      \n",ov_frame);
//			ov_frame=0;
//		//	LCD_ShowString(0,0,16,tt ,0);
//    }
//}


/**********************************************************/
/*            						ADC						                  */
/**********************************************************/


/**********************************************************/
/*            						DAC						                  */
/**********************************************************/



/**********************************************************/
/*            						DMA								              */
/**********************************************************/
extern UART_HandleTypeDef UART1_Handler; //UART句柄
DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA句柄
extern SemaphoreHandle_t DMATCIE_Sem;//DMA数据是否发送完成信号量
void MYDMA_Config(void)
{ 
    __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
    
    //Tx DMA配置
    UART1TxDMA_Handler.Instance=DMA2_Stream7;                            //数据流选择
    UART1TxDMA_Handler.Init.Channel=DMA_CHANNEL_4;                                //通道选择
    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设普通模式
    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;               //高优先级
    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
    HAL_DMA_DeInit(&UART1TxDMA_Handler);   
    HAL_DMA_Init(&UART1TxDMA_Handler);
	
	DMA2_Stream7->CR |= DMA_SxCR_TCIE;//使能传输完成中断
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);          //开启DMA2_Stream7中断  
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,2,0);	
	UART1_Handler.Instance->CR3 |= USART_CR3_DMAT;//使能串口DMA发送
}
//开启一次DMA传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
void MYDMA_USART_Transmit(uint8_t *pData, uint16_t Size)
{

	//HAL_DMA_Start(UART1_Handler.hdmatx, (u32)pData, (uint32_t)&UART1_Handler.Instance->DR, Size);//开启DMA传输
	//HAL库坑爹 让我来亲自执行
	  __HAL_DMA_DISABLE(UART1_Handler.hdmatx);
  UART1_Handler.hdmatx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
  UART1_Handler.hdmatx->Instance->NDTR = Size;
  UART1_Handler.hdmatx->Instance->PAR = (uint32_t)&UART1_Handler.Instance->DR;
  UART1_Handler.hdmatx->Instance->M0AR = (u32)pData;
  __HAL_DMA_ENABLE(UART1_Handler.hdmatx);
}	

void DMA2_Stream7_IRQHandler(void)
{
	BaseType_t tmp;
	if(DMA2->HISR|DMA_HISR_TCIF7)//数据流 7 传输完成中断标志
	{
//		UART1_Handler.Instance->CR3 &= ~USART_CR3_DMAT;//DISABLE串口DMA发送
		xSemaphoreGiveFromISR(DMATCIE_Sem,&tmp);//DMA发送完成
		if(tmp == pdTRUE)
		{
			portYIELD_FROM_ISR(tmp);
		}
	}
	DMA2->HIFCR = DMA2->HISR;
}


/**********************************************************/
/*            						SPI/I2S						              */
/**********************************************************/




/**********************************************************/
/*            					CRC32						                  */
/**********************************************************/
CRC_HandleTypeDef CRC_Handler;
void crc32_init(void)
{
	CRC_Handler.Instance=CRC;
	HAL_CRC_Init(&CRC_Handler);
	__HAL_RCC_CRC_CLK_ENABLE();
}

u32 calc_crc32(u32 *pBuffer,u32 BufferLength)
{
	return HAL_CRC_Calculate(&CRC_Handler, pBuffer, BufferLength);
}

//在线转换 http://www.ip33.com/crc.html
u32 crc32_m480(u32 *pBuffer)//字节反转 兼容m480及网页
{
	u8 temp[28];
	u8 *old=(u8*)pBuffer;
	u8 i;
	for(i=0;i<7;i++)
	{
		temp[0+4*i]=old[3+4*i];
		temp[1+4*i]=old[2+4*i];
		temp[2+4*i]=old[1+4*i];
		temp[3+4*i]=old[0+4*i];
	}
	return calc_crc32((u32*)temp,7);
}

/**********************************************************/
/*            					OTHER						                  */
/**********************************************************/

float mapf(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
	return(val/(I_Max-I_Min)*(O_Max-O_Min) + O_Min);
}


/**********************************************************/
/*            						SYS								              */
/**********************************************************/

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//系统时钟初始化	
//包括时钟设置/中断管理/GPIO设置等
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/5
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
////////////////////////////////////////////////////////////////////////////////// 

//时钟系统配置函数
//Fvco=Fs*(plln/pllm);
//SYSCLK=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCO频率
//SYSCLK:系统时钟频率
//Fusb:USB,SDIO,RNG等的时钟频率
//Fs:PLL输入时钟频率,可以是HSI,HSE等. 
//plln:主PLL倍频系数(PLL倍频),取值范围:64~432.
//pllm:主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
//pllp:系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
//pllq:USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.

//外部晶振为25M的时候,推荐值:plln=360,pllm=25,pllp=2,pllq=8.
//得到:Fvco=25*(360/25)=360Mhz
//     SYSCLK=360/2=180Mhz
//     Fusb=360/8=45Mhz
//返回值:0,成功;1,失败
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef RCC_OscInitStructure; 
    RCC_ClkInitTypeDef RCC_ClkInitStructure;
    
    __HAL_RCC_PWR_CLK_ENABLE(); //使能PWR时钟
    
    //下面这个设置用来设置调压器输出电压级别，以便在器件未以最大频率工作
    //时使性能与功耗实现平衡，此功能只有STM32F42xx和STM32F43xx器件有，
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//设置调压器输出电压级别1
    
    RCC_OscInitStructure.OscillatorType=RCC_OSCILLATORTYPE_HSE;    //时钟源为HSE
    RCC_OscInitStructure.HSEState=RCC_HSE_ON;                      //打开HSE
    RCC_OscInitStructure.PLL.PLLState=RCC_PLL_ON;//打开PLL
    RCC_OscInitStructure.PLL.PLLSource=RCC_PLLSOURCE_HSE;//PLL时钟源选择HSE
    RCC_OscInitStructure.PLL.PLLM=pllm; //主PLL和音频PLL分频系数(PLL之前的分频),取值范围:2~63.
    RCC_OscInitStructure.PLL.PLLN=plln; //主PLL倍频系数(PLL倍频),取值范围:64~432.  
    RCC_OscInitStructure.PLL.PLLP=pllp; //系统时钟的主PLL分频系数(PLL之后的分频),取值范围:2,4,6,8.(仅限这4个值!)
    RCC_OscInitStructure.PLL.PLLQ=pllq; //USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频),取值范围:2~15.
    ret=HAL_RCC_OscConfig(&RCC_OscInitStructure);//初始化
	
    if(ret!=HAL_OK) while(1);
    
    ret=HAL_PWREx_EnableOverDrive(); //开启Over-Driver功能
    if(ret!=HAL_OK) while(1);
    
    //选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2
    RCC_ClkInitStructure.ClockType=(RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;//设置系统时钟时钟源为PLL
    RCC_ClkInitStructure.AHBCLKDivider=RCC_SYSCLK_DIV1;//AHB分频系数为1
    RCC_ClkInitStructure.APB1CLKDivider=RCC_HCLK_DIV4; //APB1分频系数为4
    RCC_ClkInitStructure.APB2CLKDivider=RCC_HCLK_DIV2; //APB2分频系数为2
    ret=HAL_RCC_ClockConfig(&RCC_ClkInitStructure,FLASH_LATENCY_5);//同时设置FLASH延时周期为5WS，也就是6个CPU周期。
		
    if(ret!=HAL_OK) while(1);
}

#ifdef  USE_FULL_ASSERT
//当编译提示出错的时候此函数用来报告错误的文件和所在行
//file：指向源文件
//line：指向在文件中的行数
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1)
	{
	}
}
#endif

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("WFI;");	  
}
//关闭所有中断(但是不包括fault和NMI中断)
void INTX_DISABLE(void)
{
	__ASM volatile("CPSID   I");
	__ASM volatile("BX     LR");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("CPSIE   I");
	__ASM volatile("BX      LR");
}
//设置栈顶地址
//addr:栈顶地址
void MSR_MSP(u32 addr) 
{
	__ASM volatile("MSR MSP, r0"); 			//set Main Stack value
	__ASM volatile("BX r14");
}

