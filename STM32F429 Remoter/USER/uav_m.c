#include "uav_m.h"
#include "delay.h"
#include "cm4.h"
/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

u8 ack = 0;
u8 isrxpid = 0;//bit7:pid1-3	bit6:pid4-6
//EEPROM_Data fram_data;

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

//void nrf_rx_callback()
//{
//	if(*(u32*)(Rx_BUF+28) == crc32_m480((u32*)Rx_BUF))
//	{
//		switch (Rx_BUF[1])
//    {
//    //	case 0x01://STATUS
//				//report_status(*(s16*)(Rx_BUF+2),*(s16*)(Rx_BUF+4),*(s16*)(Rx_BUF+6),*(s16*)(Rx_BUF+8),*(s16*)(Rx_BUF+12),*(s16*)(Rx_BUF+14));
//    //		break;
//				case 0x10://pid1
//					memcpy(&fram_data,Rx_BUF+2,18);
//					isrxpid |= 0x80;//收到了pid1-3，isrxpid bit7置1
//				break;
//				case 0x11://pid2
//					memcpy(((u8*)(&fram_data))+18,Rx_BUF+2,18);
//					isrxpid |= 0x40;//收到了pid4-6，isrxpid bit6置1
//				break;
//    	default:
//				niming_report(Rx_BUF[1],Rx_BUF+2,Rx_BUF[0]);
//    		break;
//    }
//	}
//}

//extern QueueHandle_t NRFTx_Queue;//NRF要发送的数据队列
//  len	fun data..........	crc32
//	0		1		2								28 29 30 31 byte
//void add_crc32(u8* Tx_BUF)
//{
//	memset(Tx_BUF,0,32);
//	Tx_BUF[0] = len+6;
//	Tx_BUF[1] = fun;
//	memcpy(Tx_BUF+2,pBuf,len);
//	*(u32*)(Tx_BUF+28) = crc32_m480((u32*)Tx_BUF);
////	if(needack)
////	{
////		while(ack != fun)//等待m480返回ack
////		{
////			NRF24L01_TxPacket_AP(Tx_BUF);
////			delay_ms(1);
////		}
////			ack = 0;
////	}
////	else
////	{
////		NRF24L01_TxPacket_AP(Tx_BUF);
////	}
//	xQueueSend(NRFTx_Queue,Tx_BUF,portMAX_DELAY);
//}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case GPIO_PIN_2:
						delay_xms(20);      //消抖
						if(PEin(2)==0)
						{
							LED1=!LED1;
						}
            break;
        case GPIO_PIN_3:
						delay_xms(20);      //消抖
						if(PEin(3)==0)
						{
							LED2=!LED2;
						}
            break;
        case GPIO_PIN_4:
						delay_xms(20);      //消抖
						if(PEin(4)==0)
						{
							LED3=!LED3;
						}
				case GPIO_PIN_8:	//NRF24L01_IRQ
						if(PGin(8)==0)
						{
//							if(NRF24L01_Check_Event() == 0)
//							{
//								isrxdat = 1;//标记接收到了数据
//							}
//							if(istxrdy == 1)
//							{
//								NRF24L01_TxPacket_AP(Tx_BUF);//如果要发送的数据准备好
//								istxrdy = 0;
//							}
						}
            break;
    }
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//1ms
//{
//	static u8 i;
//    if(htim->Instance==TIM6)
//    {
//			if(NRF24L01_Check_Event() == 0)
//			{
//				isrxdat = 1;//标记接收到了数据
//				if(Rx_BUF[1] == 0xFF)
//				{
//					ack = Rx_BUF[2];
//				}
//				nrf_rx_callback();
//				isrxdat = 0;
//			}
//			if(i == 10)//10ms发一次遥控数据
//			{
//				nrf_tx_callback(sizeof(NRF24L01_DATA),0x00,(u8*)&send_data,0);
//				i = 0;
//			}
//			i++;
//			LED3=!LED3;
//    }
//}










