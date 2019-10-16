#include "cm4.h"
#include "delay.h"
#include "usart.h"
#include "cm4.h"
#include "malloc.h"
#include "sdio_sdcard.h"
#include "ff.h"
#include "lcd.h"
#include "GUI.h"
#include "usbh_usr.h" 
#include "24l01.h"
#include "report.h"
#include "uav_m.h"

/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

USBH_HOST  USB_Host;
USB_OTG_CORE_HANDLE  USB_OTG_Core_dev;
extern HID_Machine_TypeDef HID_Machine;
TaskHandle_t StartTask_Handler,USB_Handler,USBData_Handler,UARTRx_Handler,UARTTx_Handler,NRFRx_Handler,NRFTx_Handler,NRFProcess_Handler;
QueueHandle_t USBData_Queue;//USB接收到的数据队列
QueueHandle_t UARTRx_Queue;//UART接收到的数据队列
QueueHandle_t UARTTx_Queue;//UART要发送的数据队列
QueueHandle_t NRFRx_Queue;//NRF接收到的数据队列
QueueHandle_t NRFTx_Queue;//NRF要发送的数据队列
SemaphoreHandle_t DMATCIE_Sem;//DMA数据是否发送完成信号量
SemaphoreHandle_t PIDRDY_Sem;//是否从M480收到PID信号量
SemaphoreHandle_t NRFIRQ_Sem;//是否发生NRF24L01中断信号量
EEPROM_Data fram_data;//PID
NRF24L01_DATA send_data;//遥控数据



void USB_task(void *pvParameters)
{
	while(1)
	{
		USBH_Process(&USB_OTG_Core_dev, &USB_Host);
		if(bDeviceState==1)//连接建立了
		{ 
			if(USBH_Check_HIDCommDead(&USB_OTG_Core_dev,&HID_Machine))//检测USB HID通信,是否还正常? 
			{ 	    
				USBH_HID_Reconnect();//重连
			}				
			
		}else	//连接未建立的时候,检测
		{
			if(USBH_Check_EnumeDead(&USB_Host))	//检测USB HOST 枚举是否死机了?死机了,则重新初始化 
			{ 	    
				USBH_HID_Reconnect();//重连
			}			
		}
		delay_ms(5);
	}
	
}

void USBData_task(void *pvParameters)
{
//KEY ↑						: data[2] bit0
//KEY ↓						: data[2] bit1
//KEY ←						: data[2] bit2
//KEY →						: data[2] bit3
//KEY START				: data[2] bit4
//KEY BACK				: data[2] bit5
//JOYSTICK L KEY	: data[2] bit6
//JOYSTICK R KEY	: data[2] bit7

//KEY L1					: data[3] bit0
//KEY R1					: data[3] bit1
//KEY A						: data[3] bit4
//KEY B						: data[3] bit5
//KEY X						: data[3] bit6
//KEY Y						: data[3] bit7
//L2 							: data[4] 0-0xFF
//R2 							: data[5] 0-0xFF
//JOYSTICK LY			:	(s16*)(&data[8]);
//JOYSTICK LX			:	(s16*)(&data[6]);
//JOYSTICK RY			:	(s16*)(&data[12]);
//JOYSTICK RX			:	(s16*)(&data[10]);
	u8 USBData[20],i;
	u8 NRF_TX_Buf[32];
	while(1)
	{
		//等待USB接收到数据
		//usbh_usr.c USR_MOUSE_ProcessData
		xQueueReceive(USBData_Queue,USBData,portMAX_DELAY);
		memset(NRF_TX_Buf,0x0,32);
		send_data.thr = (s16)mapf((float)(*(s16*)(&USBData[8]))+32768.0f,0,65535.0f,0,3000.0f);
		send_data.yaw = (s16)mapf((float)(*(s16*)(&USBData[6]))+32768.0f,0,65535.0f,0,3000.0f);
		send_data.pit = (s16)mapf((float)(*(s16*)(&USBData[12]))+32768.0f,0,65535.0f,0,3000.0f);
		send_data.rol = (s16)mapf((float)(*(s16*)(&USBData[10]))+32768.0f,0,65535.0f,0,3000.0f);
		if(send_data.thr<1500)
			send_data.thr=1500;
		send_data.thr = (s16)mapf((float)send_data.thr,1500.0f,3000.0f,0,3000.0f)-3000;
		send_data.keyA = (USBData[3]&0x10)>>4;
		send_data.keyB = (USBData[3]&0x20)>>5;
		send_data.keyX = (USBData[3]&0x40)>>6;
		send_data.keyY = (USBData[3]&0x80)>>7;
		NRF_TX_Buf[0] = sizeof(send_data);							//LEN
		NRF_TX_Buf[1] = 0x00;						//FUN
		memcpy(NRF_TX_Buf+2,&send_data,sizeof(send_data));
		*(u32*)(NRF_TX_Buf+28) = crc32_m480((u32*)NRF_TX_Buf);
		xQueueSend(NRFTx_Queue,NRF_TX_Buf,portMAX_DELAY);
		//printf("thr = %d   yaw = %d   pit = %d   rol = %d\n",send_data.thr,send_data.yaw,send_data.pit,send_data.rol);
	}
}


void UARTRx_task(void *pvParameters)
 {
	u8 NRF_TX_Buf[32];
	u8 UARTData[32];
	u8 i,fun,sum,len;
	while(1)
	{
		//等待UART接收到数据
		//USART1_IRQHandler在usart.c
		xQueueReceive(UARTRx_Queue,UARTData,portMAX_DELAY);
		memset(NRF_TX_Buf,0x0,32);
		sum = 0;//清0SUM
		fun = UARTData[2];
		len = UARTData[3];
		switch (fun)
		{
  	case 0x01:			//CONMAND
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x01,sum);
				NRF_TX_Buf[0] = 1;							//LEN
				NRF_TX_Buf[1] = 0X01;						//FUN
				NRF_TX_Buf[2] = *(UARTData+4);	//CMD1	0x01:ACC校准	0x02//GYRO校准 0x03:MAG校准
				*(u32*)(NRF_TX_Buf+28) = crc32_m480((u32*)NRF_TX_Buf);
				xQueueSend(NRFTx_Queue,NRF_TX_Buf,portMAX_DELAY);
  		break;
  	case 0x02:		//ACK
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x02,sum);
				switch (*(UARTData+4))
        {
        	case 0x01:	//读取PID请求
					NRF_TX_Buf[0] = 1;							//LEN
					NRF_TX_Buf[1] = 0X02;						//FUN
					NRF_TX_Buf[2] = *(UARTData+4);	//CMD2	0x01:读取PID请求
					*(u32*)(NRF_TX_Buf+28) = crc32_m480((u32*)NRF_TX_Buf);
					xQueueSend(NRFTx_Queue,NRF_TX_Buf,portMAX_DELAY);
					if(xSemaphoreTake(PIDRDY_Sem,100) == pdTRUE)//等待从M480读取到PID
					{
						report_pid(&fram_data);//发送pid到pc
					}
        		break;
        	default:
        		break;
        }
  		break;
		case 0x10:		//PID1-3
				fram_data.ROLPID_P=*(UARTData+4)<<8|*(UARTData+5);//保存PC发来的PID保存
				fram_data.ROLPID_I=*(UARTData+6)<<8|*(UARTData+7);
				fram_data.ROLPID_D=*(UARTData+8)<<8|*(UARTData+9);
				fram_data.PITPID_P=*(UARTData+10)<<8|*(UARTData+11);
				fram_data.PITPID_I=*(UARTData+12)<<8|*(UARTData+13);
				fram_data.PITPID_D=*(UARTData+14)<<8|*(UARTData+15);
				fram_data.YAWPID_P=*(UARTData+16)<<8|*(UARTData+17);
				fram_data.YAWPID_I=*(UARTData+18)<<8|*(UARTData+19);
				fram_data.YAWPID_D=*(UARTData+20)<<8|*(UARTData+21);
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x10,sum);
			break;
		case 0x11:		//PID4-6
				fram_data.PID4_P=*(UARTData+4)<<8|*(UARTData+5);//保存PC发来的PID保存
				fram_data.PID4_I=*(UARTData+6)<<8|*(UARTData+7);
				fram_data.PID4_D=*(UARTData+8)<<8|*(UARTData+9);
				fram_data.PID5_P=*(UARTData+10)<<8|*(UARTData+11);
				fram_data.PID5_I=*(UARTData+12)<<8|*(UARTData+13);
				fram_data.PID5_D=*(UARTData+14)<<8|*(UARTData+15);
				fram_data.PID6_P=*(UARTData+16)<<8|*(UARTData+17);
				fram_data.PID6_I=*(UARTData+18)<<8|*(UARTData+19);
				fram_data.PID6_D=*(UARTData+20)<<8|*(UARTData+21);
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x11,sum);
			break;
		case 0x12:		//PID7-9
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x12,sum);
			break;
		case 0x13:		//PID10-12
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x13,sum);
			break;
		case 0x14:		//PID13-15
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x14,sum);
			break;
		case 0x15:		//PID16-18
				for(i=0;i<len+4;i++)
				{
					sum+=*(UARTData+i);
				}
				report_check(0x15,sum);
//				FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));
//				FRAM_Read(0,(u8*)&fram_data,sizeof(fram_data));
//				FRAM_Read(0,(u8*)&fram_data,sizeof(fram_data));
//				PID_update();
				NRF_TX_Buf[0] = 18;							//LEN
				NRF_TX_Buf[1] = 0X10;						//FUN
				memcpy(NRF_TX_Buf+2,&fram_data,18);											//PID1-3
				*(u32*)(NRF_TX_Buf+28) = crc32_m480((u32*)NRF_TX_Buf);
				xQueueSend(NRFTx_Queue,NRF_TX_Buf,portMAX_DELAY);
				memset(NRF_TX_Buf,0x0,32);
				NRF_TX_Buf[0] = 18;							//LEN
				NRF_TX_Buf[1] = 0X11;						//FUN
				memcpy(NRF_TX_Buf+2,((u8*)&fram_data)+18,18);						//PID4-6
				*(u32*)(NRF_TX_Buf+28) = crc32_m480((u32*)NRF_TX_Buf);
				xQueueSend(NRFTx_Queue,NRF_TX_Buf,portMAX_DELAY);	
			break;
			break;
  	default:
  		break;
		}
	}
}


void UARTTx_task(void *pvParameters)
{
	u8 UARTData[32];
	while(1)
	{
		//等待UART要发送的数据到来
		//report.c niming_report
		xQueueReceive(UARTTx_Queue,UARTData,portMAX_DELAY);
		MYDMA_USART_Transmit(UARTData,UARTData[3]+5);//DMA发送数据
		xSemaphoreTake(DMATCIE_Sem,portMAX_DELAY);//等待发送完成 DMA2_Stream7_IRQHandler在cm4.c
	}
}

void NRFRx_task(void *pvParameters)
{
	u8 i = 0;
	u8 sta;
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
//			if( i > 5)//NRF24L01坑爹，有时收到数据不中断，先手动读取5次数据后再切换中断接收
//			{
//				xSemaphoreTake(NRFIRQ_Sem,portMAX_DELAY);//等待接收到数据
//			}
//			else
//			{
//				if(i == 5)
//				{
//					HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);  
//				}
				delay_ms(2);
//				i++;
//			}
			sta = NRF24L01_Read_Reg(STATUSS);  //读取状态寄存器的值;
			if(sta & RX_OK) //接收到数据
			{
					vTaskSuspendAll();//挂起调度器
					NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_BUF,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
					xTaskResumeAll();//恢复调度器
					xQueueSend(NRFRx_Queue,Rx_BUF,portMAX_DELAY);//向队列中发送接收到数据
			}
			if(sta & MAX_TX)
			{
				if(sta & 0x01)	//TX FIFO FULL
					NRF24L01_Write_Reg(FLUSH_TX,0xff);
			}
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUSS,sta); //清除TX_DS或MAX_RT中断标志
	}
}

void EXTI9_5_IRQHandler(void)//NRF24L01
{
//	BaseType_t tmp;
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
//	xSemaphoreGiveFromISR(NRFIRQ_Sem,&tmp);//标记接收到数据
//	if(tmp == pdTRUE)
//	{
//		portYIELD_FROM_ISR(tmp);
//	}
}

void NRFTx_task(void *pvParameters)
{
	u8 Tx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRFTx_Queue,Tx_BUF,portMAX_DELAY);////等待NRF24L01要发送的数据到来
		vTaskSuspendAll();//挂起调度器
		NRF24L01_TxPacket_AP(Tx_BUF);
		xTaskResumeAll();//恢复调度器
	}
}

void NRFProcess_task(void *pvParameters)//NRF24L01接收到数据处理
{
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRFRx_Queue,Rx_BUF,portMAX_DELAY);//等待NRF24L01接收到从M480发来的数据
		if(*(u32*)(Rx_BUF+28) == crc32_m480((u32*)Rx_BUF))
		{
			switch (Rx_BUF[1])
			{
				case 0x01://STATUS
					//report_status(*(s16*)(Rx_BUF+2),*(s16*)(Rx_BUF+4),*(s16*)(Rx_BUF+6),*(s16*)(Rx_BUF+8),*(s16*)(Rx_BUF+12),*(s16*)(Rx_BUF+14));
				niming_report(0x01,Rx_BUF+2,12);
					break;
				case 0x02://SENSER
					niming_report(0x02,Rx_BUF+2,18);
					break;
					case 0x10://pid1
						memcpy(&fram_data,Rx_BUF+2,18);
					break;
					case 0x11://pid2
						memcpy(((u8*)(&fram_data))+18,Rx_BUF+2,18);
						//收到了pid4-6
						xSemaphoreGive(PIDRDY_Sem);
					break;
					case 0x03://RCDATA
						report_rcdata(send_data.thr,send_data.yaw,send_data.rol,send_data.pit,send_data.keyA*3000,send_data.keyB*3000,send_data.keyX*3000,send_data.keyY*3000,1500,1500);
					break;
					case 0x05://POWER
						report_power(*(u16*)(Rx_BUF+2),*(u16*)(Rx_BUF+4));
					break;
				default:
				//	niming_report(Rx_BUF[1],Rx_BUF+2,Rx_BUF[0]);
					break;
			}
		}
	}
}







void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建USB任务
  xTaskCreate((TaskFunction_t )USB_task,            //任务函数
                (const char*    )"USB_task",          //任务名称
                (uint16_t       )400,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )6,       //任务优先级
                (TaskHandle_t*  )&USB_Handler);   //任务句柄       
	//创建USB数据处理任务
  xTaskCreate((TaskFunction_t )USBData_task,            //任务函数
                (const char*    )"USBData_task",          //任务名称
                (uint16_t       )400,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )5,       //任务优先级
                (TaskHandle_t*  )&USBData_Handler);   //任务句柄       
	//创建UART接收任务
  xTaskCreate((TaskFunction_t )UARTRx_task,            //任务函数
                (const char*    )"UARTRx_task",          //任务名称
                (uint16_t       )400,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )4,       //任务优先级
                (TaskHandle_t*  )&UARTRx_Handler);   //任务句柄    
	//创建UART发送任务
  xTaskCreate((TaskFunction_t )UARTTx_task,            //任务函数
                (const char*    )"UARTTx_task",          //任务名称
                (uint16_t       )400,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )4,       //任务优先级
                (TaskHandle_t*  )&UARTTx_Handler);   //任务句柄   		
	//创建NRF24L01接收任务
  xTaskCreate((TaskFunction_t )NRFRx_task,            //任务函数
                (const char*    )"NRFRx_task",          //任务名称
                (uint16_t       )200,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )5,       //任务优先级
                (TaskHandle_t*  )&NRFRx_Handler);   //任务句柄   	
	//创建NRF24L01发送任务
  xTaskCreate((TaskFunction_t )NRFTx_task,            //任务函数
                (const char*    )"NRFTx_task",          //任务名称
                (uint16_t       )200,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )5,       //任务优先级
                (TaskHandle_t*  )&NRFTx_Handler);   //任务句柄   		
	//创建NRF24L01接收数据处理任务
  xTaskCreate((TaskFunction_t )NRFProcess_task,            //任务函数
                (const char*    )"NRFProcess_task",          //任务名称
                (uint16_t       )400,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )4,       //任务优先级
                (TaskHandle_t*  )&NRFProcess_Handler);   //任务句柄   								
									
	
	
	USBData_Queue = xQueueCreate(1,20);//创建USB数据接收队列
	UARTRx_Queue = xQueueCreate(1,32);//创建UART数据接收队列
	UARTTx_Queue = xQueueCreate(1,32);//创建UART数据发送队列
	NRFRx_Queue = xQueueCreate(1,32);
	NRFTx_Queue = xQueueCreate(1,32);
	DMATCIE_Sem = xSemaphoreCreateBinary();//创建DMA数据是否发送完成信号量			
	PIDRDY_Sem = xSemaphoreCreateBinary();
	NRFIRQ_Sem = xSemaphoreCreateBinary();
	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}


int main(void)
{
	u8 tmp[32];
	memset(tmp,0xFF,32);
	HAL_Init();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	Stm32_Clock_Init(384,12,2,8);
	delay_init(192);
	uart_init(115200);
	LED_Init();
	KEY_Init();
	LCD_Init();
	SRAM_Init();
	MYDMA_Config();
	NRF24L01_Init();
	NRF24L01_Mode(3);//接收（兼发送）
	printf("NRF24L01_Check=%d\n",NRF24L01_Check());
	NRF24L01_TxPacket(tmp);//NRF24L01要先启动一次发送
//	NRF_Reg_dump();
 	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks); 
	crc32_init();
	
	
		//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )200,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )1,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
								
								
	while(1);
}




