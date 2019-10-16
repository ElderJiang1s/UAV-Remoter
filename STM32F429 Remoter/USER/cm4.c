#include "cm4.h"
/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
//2019��3��1��23:15:01 		����
//2019��3��9��17:16:22 		���TIMER	ADC
//2019��3��23��23:54:56		���DMA
//2019��6��5��23:05:54		�޸���M485KIDAE
//2019��7��23��00:45:24		�޸���STM32F429ZGT6
//2019��10��1��21:48:43		���F429 HAL��DMA

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

//�жϷ�����
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
	
	
	Timing.AddressSetupTime = 0;//��ַ����ʱ��
  Timing.AddressHoldTime = 0;//��ַ����ʱ��
  Timing.DataSetupTime = 8;//���ݱ���ʱ��
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
  GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��   GPIO_Initure.Pull=GPIO_PULLUP;              //����
  GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
  GPIO_Initure.Alternate=GPIO_AF12_FMC;       //����ΪFMC    
  HAL_GPIO_Init(GPIOD,&GPIO_Initure);          //��ʼ��PD0,1,4,5,8~15
	
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
	TIM6_Handler.Instance=TIM6;                         //ͨ�ö�ʱ��6
  TIM6_Handler.Init.Prescaler=9000-1;                     //��Ƶϵ��
  TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
  TIM6_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
  TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
  HAL_TIM_Base_Init(&TIM6_Handler);
  HAL_TIM_Base_Start_IT(&TIM6_Handler); //ʹ�ܶ�ʱ��6�Ͷ�ʱ��6�����жϣ�TIM_IT_UPDATE  
}
//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();            //ʹ��TIM6ʱ��
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn,0,1);    //�����ж����ȼ�����ռ���ȼ�0�������ȼ�1
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //����ITM6�ж�   
	}
}

void TIM6_DAC_IRQHandler ()
{
	HAL_TIM_IRQHandler(&TIM6_Handler);
}
extern u8 ov_frame;
//�ص���������ʱ���жϷ���������
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	u8* tt[10];
//    if(htim==(&TIM6_Handler))
//    {
//        LED1=!LED1;        //LED1��ת
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
extern UART_HandleTypeDef UART1_Handler; //UART���
DMA_HandleTypeDef  UART1TxDMA_Handler;      //DMA���
extern SemaphoreHandle_t DMATCIE_Sem;//DMA�����Ƿ�������ź���
void MYDMA_Config(void)
{ 
    __HAL_RCC_DMA2_CLK_ENABLE();//DMA2ʱ��ʹ��	
    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
    
    //Tx DMA����
    UART1TxDMA_Handler.Instance=DMA2_Stream7;                            //������ѡ��
    UART1TxDMA_Handler.Init.Channel=DMA_CHANNEL_4;                                //ͨ��ѡ��
    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //������ͨģʽ
    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;               //�����ȼ�
    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
    HAL_DMA_DeInit(&UART1TxDMA_Handler);   
    HAL_DMA_Init(&UART1TxDMA_Handler);
	
	DMA2_Stream7->CR |= DMA_SxCR_TCIE;//ʹ�ܴ�������ж�
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);          //����DMA2_Stream7�ж�  
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,2,0);	
	UART1_Handler.Instance->CR3 |= USART_CR3_DMAT;//ʹ�ܴ���DMA����
}
//����һ��DMA����
//huart:���ھ��
//pData�����������ָ��
//Size:�����������
void MYDMA_USART_Transmit(uint8_t *pData, uint16_t Size)
{

	//HAL_DMA_Start(UART1_Handler.hdmatx, (u32)pData, (uint32_t)&UART1_Handler.Instance->DR, Size);//����DMA����
	//HAL��ӵ� ����������ִ��
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
	if(DMA2->HISR|DMA_HISR_TCIF7)//������ 7 ��������жϱ�־
	{
//		UART1_Handler.Instance->CR3 &= ~USART_CR3_DMAT;//DISABLE����DMA����
		xSemaphoreGiveFromISR(DMATCIE_Sem,&tmp);//DMA�������
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

//����ת�� http://www.ip33.com/crc.html
u32 crc32_m480(u32 *pBuffer)//�ֽڷ�ת ����m480����ҳ
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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//ϵͳʱ�ӳ�ʼ��	
//����ʱ������/�жϹ���/GPIO���õ�
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/5
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 

//ʱ��ϵͳ���ú���
//Fvco=Fs*(plln/pllm);
//SYSCLK=Fvco/pllp=Fs*(plln/(pllm*pllp));
//Fusb=Fvco/pllq=Fs*(plln/(pllm*pllq));

//Fvco:VCOƵ��
//SYSCLK:ϵͳʱ��Ƶ��
//Fusb:USB,SDIO,RNG�ȵ�ʱ��Ƶ��
//Fs:PLL����ʱ��Ƶ��,������HSI,HSE��. 
//plln:��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.
//pllm:��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
//pllp:ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
//pllq:USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.

//�ⲿ����Ϊ25M��ʱ��,�Ƽ�ֵ:plln=360,pllm=25,pllp=2,pllq=8.
//�õ�:Fvco=25*(360/25)=360Mhz
//     SYSCLK=360/2=180Mhz
//     Fusb=360/8=45Mhz
//����ֵ:0,�ɹ�;1,ʧ��
void Stm32_Clock_Init(u32 plln,u32 pllm,u32 pllp,u32 pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef RCC_OscInitStructure; 
    RCC_ClkInitTypeDef RCC_ClkInitStructure;
    
    __HAL_RCC_PWR_CLK_ENABLE(); //ʹ��PWRʱ��
    
    //������������������õ�ѹ�������ѹ�����Ա�������δ�����Ƶ�ʹ���
    //ʱʹ�����빦��ʵ��ƽ�⣬�˹���ֻ��STM32F42xx��STM32F43xx�����У�
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//���õ�ѹ�������ѹ����1
    
    RCC_OscInitStructure.OscillatorType=RCC_OSCILLATORTYPE_HSE;    //ʱ��ԴΪHSE
    RCC_OscInitStructure.HSEState=RCC_HSE_ON;                      //��HSE
    RCC_OscInitStructure.PLL.PLLState=RCC_PLL_ON;//��PLL
    RCC_OscInitStructure.PLL.PLLSource=RCC_PLLSOURCE_HSE;//PLLʱ��Դѡ��HSE
    RCC_OscInitStructure.PLL.PLLM=pllm; //��PLL����ƵPLL��Ƶϵ��(PLL֮ǰ�ķ�Ƶ),ȡֵ��Χ:2~63.
    RCC_OscInitStructure.PLL.PLLN=plln; //��PLL��Ƶϵ��(PLL��Ƶ),ȡֵ��Χ:64~432.  
    RCC_OscInitStructure.PLL.PLLP=pllp; //ϵͳʱ�ӵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2,4,6,8.(������4��ֵ!)
    RCC_OscInitStructure.PLL.PLLQ=pllq; //USB/SDIO/������������ȵ���PLL��Ƶϵ��(PLL֮��ķ�Ƶ),ȡֵ��Χ:2~15.
    ret=HAL_RCC_OscConfig(&RCC_OscInitStructure);//��ʼ��
	
    if(ret!=HAL_OK) while(1);
    
    ret=HAL_PWREx_EnableOverDrive(); //����Over-Driver����
    if(ret!=HAL_OK) while(1);
    
    //ѡ��PLL��Ϊϵͳʱ��Դ��������HCLK,PCLK1��PCLK2
    RCC_ClkInitStructure.ClockType=(RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStructure.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;//����ϵͳʱ��ʱ��ԴΪPLL
    RCC_ClkInitStructure.AHBCLKDivider=RCC_SYSCLK_DIV1;//AHB��Ƶϵ��Ϊ1
    RCC_ClkInitStructure.APB1CLKDivider=RCC_HCLK_DIV4; //APB1��Ƶϵ��Ϊ4
    RCC_ClkInitStructure.APB2CLKDivider=RCC_HCLK_DIV2; //APB2��Ƶϵ��Ϊ2
    ret=HAL_RCC_ClockConfig(&RCC_ClkInitStructure,FLASH_LATENCY_5);//ͬʱ����FLASH��ʱ����Ϊ5WS��Ҳ����6��CPU���ڡ�
		
    if(ret!=HAL_OK) while(1);
}

#ifdef  USE_FULL_ASSERT
//��������ʾ�����ʱ��˺����������������ļ���������
//file��ָ��Դ�ļ�
//line��ָ�����ļ��е�����
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1)
	{
	}
}
#endif

//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
void WFI_SET(void)
{
	__ASM volatile("WFI;");	  
}
//�ر������ж�(���ǲ�����fault��NMI�ж�)
void INTX_DISABLE(void)
{
	__ASM volatile("CPSID   I");
	__ASM volatile("BX     LR");
}
//���������ж�
void INTX_ENABLE(void)
{
	__ASM volatile("CPSIE   I");
	__ASM volatile("BX      LR");
}
//����ջ����ַ
//addr:ջ����ַ
void MSR_MSP(u32 addr) 
{
	__ASM volatile("MSR MSP, r0"); 			//set Main Stack value
	__ASM volatile("BX r14");
}

