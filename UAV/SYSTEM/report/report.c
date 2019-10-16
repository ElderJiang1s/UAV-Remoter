#include "report.h"
#include "24l01.h"
/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"



extern u8 isrxdat;



extern QueueHandle_t NRF_TX_Queue;//NRF24L01�������ݶ���
void nrf_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>26)return;	//���26�ֽ����� 
	memset(send_buf,0,32);
	send_buf[0]=len;	//֡ͷ
	send_buf[1]=fun;	//������
	memcpy(send_buf+2,data,len);//��������
	CRC->CTL |= CRC_CTL_CRCEN_Msk;
	CRC_SET_SEED(0xFFFFFFFF);
	for(i=0;i<7;i++)
	{
		CRC->DAT = *(u32*)(send_buf+i*4);
	}
	*(u32*)(send_buf+28) = CRC->CHECKSUM;
	CRC->CTL &= ~CRC_CTL_CRCEN_Msk;
	//NRF24L01_TxPacket(send_buf);
	xQueueSend(NRF_TX_Queue,send_buf,portMAX_DELAY);//��Ҫ���͵����ݷ��͵����Ͷ���
}

void niming_report(u8 fun,u8*data,u8 len)
{
	BaseType_t tmp;
	u8 send_buf[32];
	u8 i;
	if(len>27)return;	//���27�ֽ����� 
	memset(send_buf,0,32);
	send_buf[len+4]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=len;	//���ݳ���
	memcpy(send_buf+4,data,len);//��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	for(i=0;i<len+5;i++)	//��������
	{		
		while(UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = send_buf[i];
	}
}
void report_status(s16 roll,s16 pitch,s16 yaw,s32 alt,u8 mode,u8 armed)
{
	u8 data[12];
//	roll*=100;
//	pitch*=100;
//	yaw*=100;
	data[0]=(roll>>8)&0xFF;
	data[1]=roll&0xFF;
	data[2]=(pitch>>8)&0xFF;
	data[3]=pitch&0xFF;
	data[4]=(yaw>>8)&0xFF;
	data[5]=yaw&0xFF;
	data[6]=(alt>>24)&0xFF;
	data[7]=(alt>>16)&0xFF;
	data[8]=(alt>>8)&0xFF;
	data[9]=alt&0xFF;
	data[10]=mode;
	data[11]=armed;
	niming_report(0x01,data,12);//������0x01
}


//�ɻ���̬�Ȼ�����Ϣ
//alt���߶�(cm)
//mode������ģʽ
//armed��0���� 1����
void nrf_report_status(s16 roll,s16 pitch,s16 yaw,s32 alt,u8 mode,u8 armed)
{
	u8 data[12];
//	roll*=100;
//	pitch*=100;
//	yaw*=100;
	data[0]=(roll>>8)&0xFF;
	data[1]=roll&0xFF;
	data[2]=(pitch>>8)&0xFF;
	data[3]=pitch&0xFF;
	data[4]=(yaw>>8)&0xFF;
	data[5]=yaw&0xFF;
	data[6]=(alt>>24)&0xFF;
	data[7]=(alt>>16)&0xFF;
	data[8]=(alt>>8)&0xFF;
	data[9]=alt&0xFF;
	data[10]=mode;
	data[11]=armed;
	nrf_report(0x01,data,12);//������0x01
}




//�ɻ�����������
void nrf_report_senser(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 magx,s16 magy,s16 magz)
{
	u8 data[18];
	data[0]=(accx>>8)&0xFF;
	data[1]=accx&0xFF;
	data[2]=(accy>>8)&0xFF;
	data[3]=accy&0xFF;
	data[4]=(accz>>8)&0xFF;
	data[5]=accz&0xFF;
	data[6]=(gyrox>>8)&0xFF;
	data[7]=gyrox&0xFF;
	data[8]=(gyroy>>8)&0xFF;
	data[9]=gyroy&0xFF;
	data[10]=(gyroz>>8)&0xFF;
	data[11]=gyroz&0xFF;
	data[12]=(magx>>8)&0xFF;
	data[13]=magx&0xFF;
	data[14]=(magy>>8)&0xFF;
	data[15]=magy&0xFF;
	data[16]=(magz>>8)&0xFF;
	data[17]=magz&0xFF;
	nrf_report(0x02,data,18);//������0x02
}
//�ɻ�����������
void report_senser(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 magx,s16 magy,s16 magz)
{
	u8 data[18];
	data[0]=(accx>>8)&0xFF;
	data[1]=accx&0xFF;
	data[2]=(accy>>8)&0xFF;
	data[3]=accy&0xFF;
	data[4]=(accz>>8)&0xFF;
	data[5]=accz&0xFF;
	data[6]=(gyrox>>8)&0xFF;
	data[7]=gyrox&0xFF;
	data[8]=(gyroy>>8)&0xFF;
	data[9]=gyroy&0xFF;
	data[10]=(gyroz>>8)&0xFF;
	data[11]=gyroz&0xFF;
	data[12]=(magx>>8)&0xFF;
	data[13]=magx&0xFF;
	data[14]=(magy>>8)&0xFF;
	data[15]=magy&0xFF;
	data[16]=(magz>>8)&0xFF;
	data[17]=magz&0xFF;
	niming_report(0x02,data,18);//������0x02
}


//�ɻ��յ��Ŀ�������
void nrf_report_rcdata(s16 thr,s16 yaw,s16 rol,s16 pit,s16 aux1,s16 aux2,s16 aux3,s16 aux4,s16 aux5,s16 aux6)
{
	u8 data[20];
	data[0]=(thr>>8)&0xFF;
	data[1]=thr&0xFF;
	data[2]=(yaw>>8)&0xFF;
	data[3]=yaw&0xFF;
	data[4]=(rol>>8)&0xFF;
	data[5]=rol&0xFF;
	data[6]=(pit>>8)&0xFF;
	data[7]=pit&0xFF;
	data[8]=(aux1>>8)&0xFF;
	data[9]=aux1&0xFF;
	data[10]=(aux2>>8)&0xFF;
	data[11]=aux2&0xFF;
	data[12]=(aux3>>8)&0xFF;
	data[13]=aux3&0xFF;
	data[14]=(aux4>>8)&0xFF;
	data[15]=aux4&0xFF;
	data[16]=(aux5>>8)&0xFF;
	data[17]=aux5&0xFF;
	data[18]=(aux6>>8)&0xFF;
	data[19]=aux6&0xFF;
	nrf_report(0x03,data,20);//������0x03
}




void nrf_report_power(u16 voltage,u16 current)
{
	u8 data[4];
	data[0]=voltage&0xFF;
	data[1]=(voltage>>8)&0xFF;
	data[2]=current&0xFF;
	data[3]=(current>>8)&0xFF;
	nrf_report(0x05,data,4);//������0x05
}


void nrf_report_check(u8 FREAM_HEAD,u8 CHECK_SUM)
{
	u8 data[2];
	data[0]=FREAM_HEAD;
	data[1]=CHECK_SUM;
	nrf_report(0xEF,data,2);//������0xEF
}


void nrf_report_pid(EEPROM_Data *pid)
{
	u8 data[18];
	data[0]=(pid->ROLPID_P>>8)&0xFF;
	data[1]=pid->ROLPID_P&0xFF;
	data[2]=(pid->ROLPID_I>>8)&0xFF;
	data[3]=pid->ROLPID_I&0xFF;
	data[4]=(pid->ROLPID_D>>8)&0xFF;
	data[5]=pid->ROLPID_D&0xFF;
	data[6]=(pid->PITPID_P>>8)&0xFF;
	data[7]=pid->PITPID_P&0xFF;
	data[8]=(pid->PITPID_I>>8)&0xFF;
	data[9]=pid->PITPID_I&0xFF;
	data[10]=(pid->PITPID_D>>8)&0xFF;
	data[11]=pid->PITPID_D&0xFF;
	data[12]=(pid->YAWPID_P>>8)&0xFF;
	data[13]=pid->YAWPID_P&0xFF;
	data[14]=(pid->YAWPID_I>>8)&0xFF;
	data[15]=pid->YAWPID_I&0xFF;
	data[16]=(pid->YAWPID_D>>8)&0xFF;
	data[17]=pid->YAWPID_D&0xFF;
	nrf_report(0x10,data,18);//pid1
	data[0]=(pid->PID4_P>>8)&0xFF;
	data[1]=pid->PID4_P&0xFF;
	data[2]=(pid->PID4_I>>8)&0xFF;
	data[3]=pid->PID4_I&0xFF;
	data[4]=(pid->PID4_D>>8)&0xFF;
	data[5]=pid->PID4_D&0xFF;
	data[6]=(pid->PID5_P>>8)&0xFF;
	data[7]=pid->PID5_P&0xFF;
	data[8]=(pid->PID5_I>>8)&0xFF;
	data[9]=pid->PID5_I&0xFF;
	data[10]=(pid->PID5_D>>8)&0xFF;
	data[11]=pid->PID5_D&0xFF;
	data[12]=(pid->PID6_P>>8)&0xFF;
	data[13]=pid->PID6_P&0xFF;
	data[14]=(pid->PID6_I>>8)&0xFF;
	data[15]=pid->PID6_I&0xFF;
	data[16]=(pid->PID6_D>>8)&0xFF;
	data[17]=pid->PID6_D&0xFF;
	nrf_report(0x11,data,18);//pid2
	memset(data,0,18);
//	nrf_report(0x12,data,18);//pid3
//	nrf_report(0x13,data,18);//pid4
//	nrf_report(0x14,data,18);//pid5
//	nrf_report(0x15,data,18);//pid6
}



