#include "usbh_usr.h"
#include "usb_hcd_int.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_keybd.h"  
#include "delay.h"  
#include "lcd.h"  
#include "string.h"  


/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//USBH-USR 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/24
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	   
 
//表示USB连接状态
//0,没有连接;
//1,已经连接;
vu8 bDeviceState=0;		//默认没有连接  


extern USB_OTG_CORE_HANDLE USB_OTG_Core_dev;
void USBH_Msg_Show(u8 msgx);
u8 USB_FIRST_PLUGIN_FLAG=0;	//USB第一次插入标志,如果为1,说明是第一次插入


//USB信息显示
//msgx:0,USB无连接
//     1,USB键盘
//     2,USB鼠标
//     3,不支持的USB设备
void USBH_Msg_Show(u8 msgx)
{
	POINT_COLOR=RED;
	switch(msgx)
	{
		case 0:	//USB无连接
			break;
		case 1:	//USB键盘
			break;
		case 2:	//USB鼠标
			break; 		
		case 3:	//不支持的USB设备
			break; 	 
	} 
}   


//USB OTG 中断服务函数
//处理所有USB中断
void OTG_FS_IRQHandler(void)
{ 
	USBH_OTG_ISR_Handler(&USB_OTG_Core_dev);
}  

//USB HOST 用户回调函数.
USBH_Usr_cb_TypeDef USR_Callbacks =
{
  USBH_USR_Init,
  USBH_USR_DeInit,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  NULL,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};
 
//USB HOST 初始化 
void USBH_USR_Init(void)
{
	printf("USB OTG FS MSC Host\r\n");
	printf("> USB Host library started.\r\n");
	printf("  USB Host Library v2.1.0\r\n\r\n");
	
}
//检测到U盘插入
void USBH_USR_DeviceAttached(void)//U盘插入
{ 
	u8 data[] = {0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0xB2,0xE5,0xC8,0xEB,0x21,0x0D,0x0A,0x0};
	printf("%s",data);
}
//检测到U盘拔出
void USBH_USR_DeviceDisconnected (void)//U盘移除
{ 
	u8 data[] = {0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0xB0,0xCE,0xB3,0xF6,0x21,0x0D,0x0A,0x0};
	printf("%s",data);
	bDeviceState=0;	//USB设备拔出了
	USBH_Msg_Show(0);
}  
//复位从机
void USBH_USR_ResetDevice(void)
{
	u8 data[] = {0xB8,0xB4,0xCE,0xBB,0xC9,0xE8,0xB1,0xB8,0x2E,0x2E,0x2E,0x0D,0x0A,0x0};
	printf("%s",data);
}
//检测到从机速度
//DeviceSpeed:从机速度(0,1,2 / 其他)
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		u8 data[] = {0xB8,0xDF,0xCB,0xD9,0x28,0x48,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//高速(FS)USB设备!
 	}  
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		u8 data[] = {0xC8,0xAB,0xCB,0xD9,0x28,0x46,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//全速(FS)USB设备!
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		u8 data[] = {0xB5,0xCD,0xCB,0xD9,0x28,0x4C,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//低速(LS)USB设备!
	}
	else
	{
		u8 data[] = {0xC9,0xE8,0xB1,0xB8,0xB4,0xED,0xCE,0xF3,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//设备错误!
	}
}
//检测到从机的描述符
//DeviceDesc:设备描述符指针
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{ 
	USBH_DevDesc_TypeDef *hs;
	hs=DeviceDesc;   
	printf("VID: %04Xh\r\n" , (uint32_t)(*hs).idVendor); 
	printf("PID: %04Xh\r\n" , (uint32_t)(*hs).idProduct); 
}
//从机地址分配成功
void USBH_USR_DeviceAddressAssigned(void)
{
	u8 data[] = {0xB4,0xD3,0xBB,0xFA,0xB5,0xD8,0xD6,0xB7,0xB7,0xD6,0xC5,0xE4,0xB3,0xC9,0xB9,0xA6,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//从机地址分配成功!
}
//配置描述符获有效
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
	USBH_InterfaceDesc_TypeDef *id; 
	id = itfDesc;   
	if((*id).bInterfaceClass==0x08)
	{
		u8 data[] = {0xBF,0xC9,0xD2,0xC6,0xB6,0xAF,0xB4,0xE6,0xB4,0xA2,0xC6,0xF7,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//可移动存储器设备!
	}else if((*id).bInterfaceClass==0x03)
	{
		u8 data[] = {0x48,0x49,0x44,0x20,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//HID 设备!
	}    
}
//获取到设备Manufacturer String
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	printf("Manufacturer: %s\r\n",(char *)ManufacturerString);
}
//获取到设备Product String 
void USBH_USR_Product_String(void *ProductString)
{
	printf("Product: %s\r\n",(char *)ProductString);  
}
//获取到设备SerialNum String 
void USBH_USR_SerialNum_String(void *SerialNumString)
{
	printf("Serial Number: %s\r\n",(char *)SerialNumString);    
} 
//设备USB枚举完成
void USBH_USR_EnumerationDone(void)
{ 
	u8 data[] = {0xC9,0xE8,0xB1,0xB8,0xC3,0xB6,0xBE,0xD9,0xCD,0xEA,0xB3,0xC9,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//设备枚举完成!   
} 
//无法识别的USB设备
void USBH_USR_DeviceNotSupported(void)
{ 
	USBH_Msg_Show(3);//无法识别的USB设备
	u8 data[] = {0xCE,0xDE,0xB7,0xA8,0xCA,0xB6,0xB1,0xF0,0xB5,0xC4,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//无法识别的USB设备!  
}  
//等待用户输入按键,执行下一步操作
USBH_USR_Status USBH_USR_UserInput(void)
{ 
	u8 data[] = {0xCC,0xF8,0xB9,0xFD,0xD3,0xC3,0xBB,0xA7,0xC8,0xB7,0xC8,0xCF,0xB2,0xBD,0xD6,0xE8,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//跳过用户确认步骤!
	bDeviceState=1;//USB设备已经连接成功
	return USBH_USR_RESP_OK;
} 
//USB接口电流过载
void USBH_USR_OverCurrentDetected (void)
{
	u8 data[] = {0xB6,0xCB,0xBF,0xDA,0xB5,0xE7,0xC1,0xF7,0xB9,0xFD,0xB4,0xF3,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//端口电流过大!!!
}  
//重新初始化
void USBH_USR_DeInit(void)
{
	u8 data[] = {0xD6,0xD8,0xD0,0xC2,0xB3,0xF5,0xCA,0xBC,0xBB,0xAF,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//重新初始化!!!
}
//无法恢复的错误!!  
void USBH_USR_UnrecoveredError (void)
{
	u8 data[] = {0xCE,0xDE,0xB7,0xA8,0xBB,0xD6,0xB8,0xB4,0xB5,0xC4,0xB4,0xED,0xCE,0xF3,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//无法恢复的错误!!!
}
//////////////////////////////////////////////////////////////////////////////////////////
//下面两个函数,为ALIENTEK添加,以防止USB死机

//USB枚举状态死机检测,防止USB枚举失败导致的死机
//phost:USB_HOST结构体指针
//返回值:0,没有死机
//       1,死机了,外部必须重新启动USB连接.
u8 USBH_Check_EnumeDead(USBH_HOST *phost)
{
	static u16 errcnt=0;
	//这个状态,如果持续存在,则说明USB死机了.
	if(phost->gState==HOST_CTRL_XFER&&(phost->EnumState==ENUM_IDLE||phost->EnumState==ENUM_GET_FULL_DEV_DESC))
	{
		errcnt++;
		if(errcnt>2000)//死机了
		{ 
			errcnt=0;
			RCC->AHB2RSTR|=1<<7;	//USB OTG FS 复位
			delay_ms(5);
			RCC->AHB2RSTR&=~(1<<7);	//复位结束  
			return 1;
		} 
	}else errcnt=0;
	return 0;
} 
//USB HID通信死机检测,防止USB通信死机(暂时仅针对:DTERR,即Data toggle error)
//pcore:USB_OTG_Core_dev_HANDLE结构体指针
//phidm:HID_Machine_TypeDef结构体指针 
//返回值:0,没有死机
//       1,死机了,外部必须重新启动USB连接.
u8 USBH_Check_HIDCommDead(USB_OTG_CORE_HANDLE *pcore,HID_Machine_TypeDef *phidm)
{
 	if(pcore->host.HC_Status[phidm->hc_num_in]==HC_DATATGLERR)//检测到DTERR错误,直接重启USB.
	{  
		return 1;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//USB键盘鼠标数据处理

//鼠标初始化
void USR_MOUSE_Init	(void)
{
 	USBH_Msg_Show(2);		//USB 鼠标
	USB_FIRST_PLUGIN_FLAG=1;//标记第一次插入
}
//键盘初始化
void  USR_KEYBRD_Init(void)
{ 
 	USBH_Msg_Show(1);		//USB 键盘
	USB_FIRST_PLUGIN_FLAG=1;//标记第一次插入
}

//零时数组,用于存放鼠标坐标/键盘输入内容(4.3屏,最大可以输入2016字节)
u8 tbuf[2017] __attribute__((aligned (4))); 




//USB鼠标数据处理
//data:USB鼠标数据结构体指针
extern QueueHandle_t USBData_Queue;//USB接收到的数据队列
void USR_MOUSE_ProcessData(uint8_t *data)
{
	BaseType_t tmp;
	xQueueSendFromISR(USBData_Queue,data,&tmp);//发送USB接收到的数据到队列
	if(tmp == pdTRUE)
	{
		portYIELD_FROM_ISR(tmp);
	}
} 


//USB键盘数据处理
//data:USB鼠标数据结构体指针
void  USR_KEYBRD_ProcessData (uint8_t data)
{ 
	static u16 pos; 
	static u16 endx,endy;
	static u16 maxinputchar;
	
	u8 buf[4];
	if(USB_FIRST_PLUGIN_FLAG)//第一次插入,将数据清零
	{
		USB_FIRST_PLUGIN_FLAG=0;
		endx=((lcddev.width-30)/8)*8+30;		//得到endx值
		endy=((lcddev.height-220)/16)*16+220;	//得到endy值
		maxinputchar=((lcddev.width-30)/8);
		maxinputchar*=(lcddev.height-220)/16;	//当前LCD最大可以显示的字符数.
 		pos=0; 
	}
//	POINT_COLOR=BLUE;
//	sprintf((char*)buf,"%02X",data);
//	LCD_ShowString(30+56,180,200,16,16,buf);//显示键值	 
//	if(data>=' '&&data<='~')
//	{
//		tbuf[pos++]=data;
//		tbuf[pos]=0;		//添加结束符. 
//		if(pos>maxinputchar)pos=maxinputchar;//最大输入这么多
//	}else if(data==0X0D)	//退格键
//	{
//		if(pos)pos--;
//		tbuf[pos]=0;		//添加结束符.  
//	}
//	if(pos<=maxinputchar)	//没有超过显示区
//	{
//		LCD_Fill(30,220,endx,endy,WHITE);
//		LCD_ShowString(30,220,endx-30,endy-220,16,tbuf);
//	}		
	printf("KEY Board Value:%02X\r\n",data);
	printf("KEY Board Char:%c\r\n",data); 
}


extern USBH_HOST  USB_Host;

//HID重新连接
void USBH_HID_Reconnect(void)
{
	//关闭之前的连接
	USBH_DeInit(&USB_OTG_Core_dev,&USB_Host);	//复位USB HOST
	USB_OTG_StopHost(&USB_OTG_Core_dev);		//停止USBhost
	if(USB_Host.usr_cb->DeviceDisconnected)		//存在,才禁止
	{
		USB_Host.usr_cb->DeviceDisconnected(); 	//关闭USB连接
		USBH_DeInit(&USB_OTG_Core_dev, &USB_Host);
		USB_Host.usr_cb->DeInit();
		USB_Host.class_cb->DeInit(&USB_OTG_Core_dev,&USB_Host.device_prop);
	}
	USB_OTG_DisableGlobalInt(&USB_OTG_Core_dev);//关闭所有中断
	//重新复位USB
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();//USB OTG FS 复位
	delay_ms(5);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();//复位结束
	memset(&USB_OTG_Core_dev,0,sizeof(USB_OTG_CORE_HANDLE));
	memset(&USB_Host,0,sizeof(USB_Host));
	//重新连接USB HID设备
	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);  
}








