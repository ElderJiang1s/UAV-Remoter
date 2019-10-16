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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//USBH-USR ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/24
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//�޸���Ϣ
//��
////////////////////////////////////////////////////////////////////////////////// 	   
 
//��ʾUSB����״̬
//0,û������;
//1,�Ѿ�����;
vu8 bDeviceState=0;		//Ĭ��û������  


extern USB_OTG_CORE_HANDLE USB_OTG_Core_dev;
void USBH_Msg_Show(u8 msgx);
u8 USB_FIRST_PLUGIN_FLAG=0;	//USB��һ�β����־,���Ϊ1,˵���ǵ�һ�β���


//USB��Ϣ��ʾ
//msgx:0,USB������
//     1,USB����
//     2,USB���
//     3,��֧�ֵ�USB�豸
void USBH_Msg_Show(u8 msgx)
{
	POINT_COLOR=RED;
	switch(msgx)
	{
		case 0:	//USB������
			break;
		case 1:	//USB����
			break;
		case 2:	//USB���
			break; 		
		case 3:	//��֧�ֵ�USB�豸
			break; 	 
	} 
}   


//USB OTG �жϷ�����
//��������USB�ж�
void OTG_FS_IRQHandler(void)
{ 
	USBH_OTG_ISR_Handler(&USB_OTG_Core_dev);
}  

//USB HOST �û��ص�����.
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
 
//USB HOST ��ʼ�� 
void USBH_USR_Init(void)
{
	printf("USB OTG FS MSC Host\r\n");
	printf("> USB Host library started.\r\n");
	printf("  USB Host Library v2.1.0\r\n\r\n");
	
}
//��⵽U�̲���
void USBH_USR_DeviceAttached(void)//U�̲���
{ 
	u8 data[] = {0xBC,0xEC,0xB2,0xE2,0xB5,0xBD,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0xB2,0xE5,0xC8,0xEB,0x21,0x0D,0x0A,0x0};
	printf("%s",data);
}
//��⵽U�̰γ�
void USBH_USR_DeviceDisconnected (void)//U���Ƴ�
{ 
	u8 data[] = {0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0xB0,0xCE,0xB3,0xF6,0x21,0x0D,0x0A,0x0};
	printf("%s",data);
	bDeviceState=0;	//USB�豸�γ���
	USBH_Msg_Show(0);
}  
//��λ�ӻ�
void USBH_USR_ResetDevice(void)
{
	u8 data[] = {0xB8,0xB4,0xCE,0xBB,0xC9,0xE8,0xB1,0xB8,0x2E,0x2E,0x2E,0x0D,0x0A,0x0};
	printf("%s",data);
}
//��⵽�ӻ��ٶ�
//DeviceSpeed:�ӻ��ٶ�(0,1,2 / ����)
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		u8 data[] = {0xB8,0xDF,0xCB,0xD9,0x28,0x48,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//����(FS)USB�豸!
 	}  
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		u8 data[] = {0xC8,0xAB,0xCB,0xD9,0x28,0x46,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//ȫ��(FS)USB�豸!
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		u8 data[] = {0xB5,0xCD,0xCB,0xD9,0x28,0x4C,0x53,0x29,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//����(LS)USB�豸!
	}
	else
	{
		u8 data[] = {0xC9,0xE8,0xB1,0xB8,0xB4,0xED,0xCE,0xF3,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//�豸����!
	}
}
//��⵽�ӻ���������
//DeviceDesc:�豸������ָ��
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{ 
	USBH_DevDesc_TypeDef *hs;
	hs=DeviceDesc;   
	printf("VID: %04Xh\r\n" , (uint32_t)(*hs).idVendor); 
	printf("PID: %04Xh\r\n" , (uint32_t)(*hs).idProduct); 
}
//�ӻ���ַ����ɹ�
void USBH_USR_DeviceAddressAssigned(void)
{
	u8 data[] = {0xB4,0xD3,0xBB,0xFA,0xB5,0xD8,0xD6,0xB7,0xB7,0xD6,0xC5,0xE4,0xB3,0xC9,0xB9,0xA6,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�ӻ���ַ����ɹ�!
}
//��������������Ч
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
	USBH_InterfaceDesc_TypeDef *id; 
	id = itfDesc;   
	if((*id).bInterfaceClass==0x08)
	{
		u8 data[] = {0xBF,0xC9,0xD2,0xC6,0xB6,0xAF,0xB4,0xE6,0xB4,0xA2,0xC6,0xF7,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//���ƶ��洢���豸!
	}else if((*id).bInterfaceClass==0x03)
	{
		u8 data[] = {0x48,0x49,0x44,0x20,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
		printf("%s",data);//HID �豸!
	}    
}
//��ȡ���豸Manufacturer String
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	printf("Manufacturer: %s\r\n",(char *)ManufacturerString);
}
//��ȡ���豸Product String 
void USBH_USR_Product_String(void *ProductString)
{
	printf("Product: %s\r\n",(char *)ProductString);  
}
//��ȡ���豸SerialNum String 
void USBH_USR_SerialNum_String(void *SerialNumString)
{
	printf("Serial Number: %s\r\n",(char *)SerialNumString);    
} 
//�豸USBö�����
void USBH_USR_EnumerationDone(void)
{ 
	u8 data[] = {0xC9,0xE8,0xB1,0xB8,0xC3,0xB6,0xBE,0xD9,0xCD,0xEA,0xB3,0xC9,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�豸ö�����!   
} 
//�޷�ʶ���USB�豸
void USBH_USR_DeviceNotSupported(void)
{ 
	USBH_Msg_Show(3);//�޷�ʶ���USB�豸
	u8 data[] = {0xCE,0xDE,0xB7,0xA8,0xCA,0xB6,0xB1,0xF0,0xB5,0xC4,0x55,0x53,0x42,0xC9,0xE8,0xB1,0xB8,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�޷�ʶ���USB�豸!  
}  
//�ȴ��û����밴��,ִ����һ������
USBH_USR_Status USBH_USR_UserInput(void)
{ 
	u8 data[] = {0xCC,0xF8,0xB9,0xFD,0xD3,0xC3,0xBB,0xA7,0xC8,0xB7,0xC8,0xCF,0xB2,0xBD,0xD6,0xE8,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�����û�ȷ�ϲ���!
	bDeviceState=1;//USB�豸�Ѿ����ӳɹ�
	return USBH_USR_RESP_OK;
} 
//USB�ӿڵ�������
void USBH_USR_OverCurrentDetected (void)
{
	u8 data[] = {0xB6,0xCB,0xBF,0xDA,0xB5,0xE7,0xC1,0xF7,0xB9,0xFD,0xB4,0xF3,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�˿ڵ�������!!!
}  
//���³�ʼ��
void USBH_USR_DeInit(void)
{
	u8 data[] = {0xD6,0xD8,0xD0,0xC2,0xB3,0xF5,0xCA,0xBC,0xBB,0xAF,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//���³�ʼ��!!!
}
//�޷��ָ��Ĵ���!!  
void USBH_USR_UnrecoveredError (void)
{
	u8 data[] = {0xCE,0xDE,0xB7,0xA8,0xBB,0xD6,0xB8,0xB4,0xB5,0xC4,0xB4,0xED,0xCE,0xF3,0x21,0x21,0x21,0x0D,0x0A,0x0};
	printf("%s",data);//�޷��ָ��Ĵ���!!!
}
//////////////////////////////////////////////////////////////////////////////////////////
//������������,ΪALIENTEK���,�Է�ֹUSB����

//USBö��״̬�������,��ֹUSBö��ʧ�ܵ��µ�����
//phost:USB_HOST�ṹ��ָ��
//����ֵ:0,û������
//       1,������,�ⲿ������������USB����.
u8 USBH_Check_EnumeDead(USBH_HOST *phost)
{
	static u16 errcnt=0;
	//���״̬,�����������,��˵��USB������.
	if(phost->gState==HOST_CTRL_XFER&&(phost->EnumState==ENUM_IDLE||phost->EnumState==ENUM_GET_FULL_DEV_DESC))
	{
		errcnt++;
		if(errcnt>2000)//������
		{ 
			errcnt=0;
			RCC->AHB2RSTR|=1<<7;	//USB OTG FS ��λ
			delay_ms(5);
			RCC->AHB2RSTR&=~(1<<7);	//��λ����  
			return 1;
		} 
	}else errcnt=0;
	return 0;
} 
//USB HIDͨ���������,��ֹUSBͨ������(��ʱ�����:DTERR,��Data toggle error)
//pcore:USB_OTG_Core_dev_HANDLE�ṹ��ָ��
//phidm:HID_Machine_TypeDef�ṹ��ָ�� 
//����ֵ:0,û������
//       1,������,�ⲿ������������USB����.
u8 USBH_Check_HIDCommDead(USB_OTG_CORE_HANDLE *pcore,HID_Machine_TypeDef *phidm)
{
 	if(pcore->host.HC_Status[phidm->hc_num_in]==HC_DATATGLERR)//��⵽DTERR����,ֱ������USB.
	{  
		return 1;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//USB����������ݴ���

//����ʼ��
void USR_MOUSE_Init	(void)
{
 	USBH_Msg_Show(2);		//USB ���
	USB_FIRST_PLUGIN_FLAG=1;//��ǵ�һ�β���
}
//���̳�ʼ��
void  USR_KEYBRD_Init(void)
{ 
 	USBH_Msg_Show(1);		//USB ����
	USB_FIRST_PLUGIN_FLAG=1;//��ǵ�һ�β���
}

//��ʱ����,���ڴ���������/������������(4.3��,����������2016�ֽ�)
u8 tbuf[2017] __attribute__((aligned (4))); 




//USB������ݴ���
//data:USB������ݽṹ��ָ��
extern QueueHandle_t USBData_Queue;//USB���յ������ݶ���
void USR_MOUSE_ProcessData(uint8_t *data)
{
	BaseType_t tmp;
	xQueueSendFromISR(USBData_Queue,data,&tmp);//����USB���յ������ݵ�����
	if(tmp == pdTRUE)
	{
		portYIELD_FROM_ISR(tmp);
	}
} 


//USB�������ݴ���
//data:USB������ݽṹ��ָ��
void  USR_KEYBRD_ProcessData (uint8_t data)
{ 
	static u16 pos; 
	static u16 endx,endy;
	static u16 maxinputchar;
	
	u8 buf[4];
	if(USB_FIRST_PLUGIN_FLAG)//��һ�β���,����������
	{
		USB_FIRST_PLUGIN_FLAG=0;
		endx=((lcddev.width-30)/8)*8+30;		//�õ�endxֵ
		endy=((lcddev.height-220)/16)*16+220;	//�õ�endyֵ
		maxinputchar=((lcddev.width-30)/8);
		maxinputchar*=(lcddev.height-220)/16;	//��ǰLCD��������ʾ���ַ���.
 		pos=0; 
	}
//	POINT_COLOR=BLUE;
//	sprintf((char*)buf,"%02X",data);
//	LCD_ShowString(30+56,180,200,16,16,buf);//��ʾ��ֵ	 
//	if(data>=' '&&data<='~')
//	{
//		tbuf[pos++]=data;
//		tbuf[pos]=0;		//��ӽ�����. 
//		if(pos>maxinputchar)pos=maxinputchar;//���������ô��
//	}else if(data==0X0D)	//�˸��
//	{
//		if(pos)pos--;
//		tbuf[pos]=0;		//��ӽ�����.  
//	}
//	if(pos<=maxinputchar)	//û�г�����ʾ��
//	{
//		LCD_Fill(30,220,endx,endy,WHITE);
//		LCD_ShowString(30,220,endx-30,endy-220,16,tbuf);
//	}		
	printf("KEY Board Value:%02X\r\n",data);
	printf("KEY Board Char:%c\r\n",data); 
}


extern USBH_HOST  USB_Host;

//HID��������
void USBH_HID_Reconnect(void)
{
	//�ر�֮ǰ������
	USBH_DeInit(&USB_OTG_Core_dev,&USB_Host);	//��λUSB HOST
	USB_OTG_StopHost(&USB_OTG_Core_dev);		//ֹͣUSBhost
	if(USB_Host.usr_cb->DeviceDisconnected)		//����,�Ž�ֹ
	{
		USB_Host.usr_cb->DeviceDisconnected(); 	//�ر�USB����
		USBH_DeInit(&USB_OTG_Core_dev, &USB_Host);
		USB_Host.usr_cb->DeInit();
		USB_Host.class_cb->DeInit(&USB_OTG_Core_dev,&USB_Host.device_prop);
	}
	USB_OTG_DisableGlobalInt(&USB_OTG_Core_dev);//�ر������ж�
	//���¸�λUSB
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();//USB OTG FS ��λ
	delay_ms(5);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();//��λ����
	memset(&USB_OTG_Core_dev,0,sizeof(USB_OTG_CORE_HANDLE));
	memset(&USB_Host,0,sizeof(USB_Host));
	//��������USB HID�豸
	USBH_Init(&USB_OTG_Core_dev,USB_OTG_FS_CORE_ID,&USB_Host,&HID_cb,&USR_Callbacks);  
}








