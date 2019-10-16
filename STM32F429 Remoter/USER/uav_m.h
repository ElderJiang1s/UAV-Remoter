#ifndef __UAV_M_H
#define __UAV_M_H
#include "cm4.h"

typedef struct
{
	u16 thr;
	u16 pit;
	u16 rol;
	u16 yaw;
	u8 keyA;
	u8 keyB;
	u8 keyX;
	u8 keyY;
}NRF24L01_DATA;//通过nrf24l01发送的遥控数据

typedef struct
{
	s16 PITPID_P;
	s16 PITPID_I;
	s16 PITPID_D;
	s16 ROLPID_P;
	s16 ROLPID_I;
	s16 ROLPID_D;
	s16 YAWPID_P;
	s16 YAWPID_I;
	s16 YAWPID_D;
	s16 PID4_P;
	s16 PID4_I;
	s16 PID4_D;
	s16 PID5_P;
	s16 PID5_I;
	s16 PID5_D;
	s16 PID6_P;
	s16 PID6_I;
	s16 PID6_D;
	float PITCH_ERR;
	float ROLL_ERR;
	float YAW_ERR;
	s16 GYROX_ERR;
	s16 GYROY_ERR;
	s16 GYROZ_ERR;
}EEPROM_Data;


extern NRF24L01_DATA send_data;

void nrf_tx_callback(u8 len,u8 fun,u8* pBuf,u8 needack);

void UART_RX_Process(u8 fun,u8 len,u8* UARTData);

#endif



