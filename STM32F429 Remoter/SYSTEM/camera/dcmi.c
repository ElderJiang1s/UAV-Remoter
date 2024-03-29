#include "cm4.h"
#include "dcmi.h" 
#include "lcd.h" 
#include "ov7670.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//DCMI驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern u16 *buf;
DCMI_HandleTypeDef  DCMI_Handler;           //DCMI句柄
DMA_HandleTypeDef   DMADMCI_Handler;        //DMA句柄

u8 ov_frame=0;  						//帧率
extern void jpeg_data_process(void);	//JPEG数据处理函数

//DCMI初始化
void DCMI_Init(void)
{
    DCMI_Handler.Instance=DCMI;
    DCMI_Handler.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;    //硬件同步HSYNC,VSYNC
    DCMI_Handler.Init.PCKPolarity=DCMI_PCKPOLARITY_RISING;  //PCLK 上升沿有效
    DCMI_Handler.Init.VSPolarity=DCMI_VSPOLARITY_HIGH;       //VSYNC 低电平有效
    DCMI_Handler.Init.HSPolarity=DCMI_HSPOLARITY_LOW;       //HSYNC 低电平有效
    DCMI_Handler.Init.CaptureRate=DCMI_CR_ALL_FRAME;        //全帧捕获
    DCMI_Handler.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B; //8位数据格式 
    HAL_DCMI_Init(&DCMI_Handler);                           //初始化DCMI，此函数会开启帧中断  
//	DCMI->CR|=DCMI_CR_CM;
	
	 DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);
	 //DCMI_DMA_Init((u32)&USART1->DR,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);
	DCMI_Start();
}

//DCMI底层驱动，引脚配置，时钟使能，中断配置
//此函数会被HAL_DCMI_Init()调用
//hdcmi:DCMI句柄
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{	
	//XCLK:PA8
	//DCMI_PCLK:PA6
	//DCMI_HREF:PA4
	//DCMI_VSYNC:PB7
	//DCMI_D0:PC6
	//DCMI_D1:PC7
	//DCMI_D2:PC8
	//DCMI_D3:PC9
	//DCMI_D4:PC11
	//DCMI_D5:PB6
	//DCMI_D6:PE5
	//DCMI_D7:PE6
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_DCMI_CLK_ENABLE();                //使能DCMI时钟

    __HAL_RCC_GPIOA_CLK_ENABLE();               //使能GPIOA时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();               //使能GPIOB时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();               //使能GPIOC时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();               //使能GPIOE时钟
 //  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_3); 
    //初始化PA4,6
    GPIO_Initure.Pin=GPIO_PIN_4|GPIO_PIN_6;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    GPIO_Initure.Alternate=GPIO_AF13_DCMI;      //复用为DCMI   
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //初始化
    
    //PB6,7
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7;  
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //初始化
    
    //PC6,7,8,9,11
    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|\
                     GPIO_PIN_9|GPIO_PIN_11;  
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //初始化
    
    //PE5,6
    GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6; 
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);         //初始化   
    
    HAL_NVIC_SetPriority(DCMI_IRQn,0,0);        //抢占优先级1，子优先级2
    HAL_NVIC_EnableIRQ(DCMI_IRQn);              //使能DCMI中断
}

//DCMI DMA配置
//mem0addr:存储器地址0  将要存储摄像头数据的内存地址(也可以是外设地址)
//mem1addr:存储器地址1  当只使用mem0addr的时候,该值必须为0
//memblen:存储器位宽,可以为:DMA_MDATAALIGN_BYTE/DMA_MDATAALIGN_HALFWORD/DMA_MDATAALIGN_WORD
//meminc:存储器增长方式,可以为:DMA_MINC_ENABLE/DMA_MINC_DISABLE
void DCMI_DMA_Init(u32 mem0addr,u32 mem1addr,u16 memsize,u32 memblen,u32 meminc)
{ 
    __HAL_RCC_DMA2_CLK_ENABLE();                                    //使能DMA2时钟
    __HAL_LINKDMA(&DCMI_Handler,DMA_Handle,DMADMCI_Handler);        //将DMA与DCMI联系起来
	
    DMADMCI_Handler.Instance=DMA2_Stream1;                          //DMA2数据流1                     
    DMADMCI_Handler.Init.Channel=DMA_CHANNEL_1;                     //通道1
    DMADMCI_Handler.Init.Direction=DMA_PERIPH_TO_MEMORY;            //外设到存储器
    DMADMCI_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                //外设非增量模式
    DMADMCI_Handler.Init.MemInc=meminc;                             //存储器增量模式
    DMADMCI_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;   //外设数据长度:32位
    DMADMCI_Handler.Init.MemDataAlignment=memblen;                  //存储器数据长度:8/16/32位
    DMADMCI_Handler.Init.Mode=DMA_CIRCULAR;   //DMA_NORMAL;//                      //使用循环模式 
    DMADMCI_Handler.Init.Priority=DMA_PRIORITY_HIGH;                //高优先级
    DMADMCI_Handler.Init.FIFOMode=DMA_FIFOMODE_ENABLE;              //使能FIFO
    DMADMCI_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_HALFFULL; //使用1/2的FIFO 
    DMADMCI_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                //存储器突发传输
    DMADMCI_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;             //外设突发单次传输 
    HAL_DMA_DeInit(&DMADMCI_Handler);                               //先清除以前的设置
    HAL_DMA_Init(&DMADMCI_Handler);	                                //初始化DMA
    
    //在开启DMA之前先使用__HAL_UNLOCK()解锁一次DMA,因为HAL_DMA_Statrt()HAL_DMAEx_MultiBufferStart()
    //这两个函数一开始要先使用__HAL_LOCK()锁定DMA,而函数__HAL_LOCK()会判断当前的DMA状态是否为锁定状态，如果是
    //锁定状态的话就直接返回HAL_BUSY，这样会导致函数HAL_DMA_Statrt()和HAL_DMAEx_MultiBufferStart()后续的DMA配置
    //程序直接被跳过！DMA也就不能正常工作，为了避免这种现象，所以在启动DMA之前先调用__HAL_UNLOC()先解锁一次DMA。
    __HAL_UNLOCK(&DMADMCI_Handler);
    if(mem1addr==0)    //开启DMA，不使用双缓冲
    {
        HAL_DMA_Start(&DMADMCI_Handler,(u32)&DCMI->DR,mem0addr,memsize);
    }
    else                //使用双缓冲
    {
        HAL_DMAEx_MultiBufferStart(&DMADMCI_Handler,(u32)&DCMI->DR,mem0addr,mem1addr,memsize);//开启双缓冲
        __HAL_DMA_ENABLE_IT(&DMADMCI_Handler,DMA_IT_TC);    //开启传输完成中断
        HAL_NVIC_SetPriority(DMA2_Stream1_IRQn,0,0);        //DMA中断优先级
        HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    }
}

//DCMI,启动传输
void DCMI_Start(void)
{  
    LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();		        //开始写入GRAM
    __HAL_DMA_ENABLE(&DMADMCI_Handler); //使能DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能
}

//DCMI,关闭传输
void DCMI_Stop(void)
{ 
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //关闭捕获
    while(DCMI->CR&0X01);               //等待传输完成
    __HAL_DMA_DISABLE(&DMADMCI_Handler);//关闭DMA
} 
u32 nline=0;
//DCMI中断服务函数
void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&DCMI_Handler);
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{//		LCD_SetCursor(0,nline); 
	nline++;
}
//捕获到一帧图像处理函数
//hdcmi:DCMI句柄
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	//jpeg_data_process();//jpeg数据处理
//	LED1=!LED1;
	nline=0;
	LCD_SetCursor(0,0); 
//	LCD_ShowString(0,0,16,"df" ,0);
 	ov_frame++; 
    //重新使能帧中断,因为HAL_DCMI_IRQHandler()函数会关闭帧中断
    __HAL_DCMI_ENABLE_IT(&DCMI_Handler,DCMI_IT_FRAME);
}

void (*dcmi_rx_callback)(void);//DCMI DMA接收回调函数
//DMA2数据流1中断服务函数
void DMA2_Stream1_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(&DMADMCI_Handler,DMA_FLAG_TCIF1_5)!=RESET)//DMA传输完成
    {
        __HAL_DMA_CLEAR_FLAG(&DMADMCI_Handler,DMA_FLAG_TCIF1_5);//清除DMA传输完成中断标志位
        dcmi_rx_callback();	//执行摄像头接收回调函数,读取数据等操作在这里面处理
    } 
}

