#include "delay.h"
#include "malloc.h"
#include "cm4.h"
#include "report.h"
/********	I2C	********/
#include "FRAM.h"
#include "bmpxxx.h"
#include "mpu6050.h"

/********	SPI	********/
#include "24l01.h"

/********	DSP	********/
#include "arm_math.h"

/********	HSUSB	******/
//#include "vcom_serial.h"

/********	AHRS	********/
#include "AHRSLib.h"
#include "uav_s.h"
#include "Filter.h"
#include "IMU.h"
#include "pid.h"

/********	RTOS	********/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"


#define OUT_MAX 3800.0f
#define OUT_MIN 0.0f
#define limit_out(in) ((in)>OUT_MAX ? OUT_MAX :((in)<OUT_MIN ? OUT_MIN : (in)))


TaskHandle_t StartTask_Handler,IMUupdate_Handler,PIDControl_Handler,NRFProcess_Handler,NRF_TX_Handler,NRF_RX_Handler,SENDState_Handler;//������
QueueHandle_t EulerData_Queue;//�ɻ���̬�Ƕȶ���
QueueHandle_t NRF_RX_Queue;//NRF24L01�������ݶ���
QueueHandle_t NRF_TX_Queue;//NRF24L01�������ݶ���


EEPROM_Data fram_data;
nrf_send RX_Data;
float set_pit = 0,set_roll = 0,set_yaw = 0;//Ҫ���õĽǶ� ��ң��������
s16 aa[3],gg[3];	//δ�˲���accel gyro
short gyro_raw[3], accel_raw[3];//�˲����accel gyro
float gyro[3];		//����
float pitch_raw,roll_raw,yaw_raw;//δ����0�ĽǶ�
PidObject PITCH_PID;
PidObject ROLL_PID;
PidObject YAW_PID;
float VBAT;
u8 uav_lock = 1;

void PID_update(void);


void IMUupdate_task(void *pvParameters)
{
	float EulerData[3];
	u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 1);		/*1ms������ʱ*/
		vTaskSuspendAll();//���������
		MPU_Get_Accelerometer(aa,aa+1,aa+2);
		MPU_Get_Gyroscope(gg,gg+1,gg+2);
		VBAT = (float)ADC_GetData(8) * 0.000805664f / 1.0518f;
		xTaskResumeAll();//�ָ�������
		ACC_IIR_Filter(aa,accel_raw);//accel�˲�
		Gyro_Filter(gg,gyro_raw);//gyro�˲�
		gyro_raw[0] -= fram_data.GYROX_ERR;
		gyro_raw[1] -= fram_data.GYROY_ERR;
		gyro_raw[2] -= fram_data.GYROZ_ERR;//gyro��ȥ���
		Get_Radian(gyro_raw,gyro);//���ٶ�����תΪ����
		IMUupdate(gyro[0],gyro[1],gyro[2],accel_raw[0],accel_raw[1],accel_raw[2],EulerData);//��̬����
		xQueueOverwrite(EulerData_Queue,EulerData);//������з��ͷɻ���̬�Ƕ���Ϣ����д
		//vTaskDelay(1);
	}
}

void PIDControl_task(void *pvParameters)
{
	float out_pit,out_roll,out_yaw;
	float EulerData[3];
	float thr;//����
	PID_update();//����PID����
	while(1)
	{
		xQueueReceive(EulerData_Queue,EulerData,portMAX_DELAY);//�Ӷ����ж�ȡ�ɻ���̬�Ƕ���Ϣ
		thr = (float)RX_Data.thr;
		set_roll = mapf(3000.0f - ((float)RX_Data.pit),0,3000.0f,-10.0f,10.0f);
		set_pit = mapf((float)RX_Data.rol,0,3000.0f,-10.0f,10.0f);
		set_yaw = mapf((float)RX_Data.yaw,0,3000.0f,-10.0f,10.0f);//����ң�����������÷��з���
		out_pit = pidUpdate(&PITCH_PID,set_pit - EulerData[0]);
		out_roll = pidUpdate(&ROLL_PID,set_roll - EulerData[1]);
		out_yaw = pidUpdate(&YAW_PID,set_yaw - EulerData[2]);//PID����
		if(RX_Data.yaw <= 30&&RX_Data.keyY&&RX_Data.keyB)
		{
			if(uav_lock == 1)
			{
				led1 = 0;
				uav_lock = 0;
			}
			else
			{
				led1 = 1;
				uav_lock = 1;
			}
		}
		if(uav_lock == 1)
		{
			led1 = 1;
			EPWM0->CMPDAT[1] = 0;
			EPWM0->CMPDAT[2] = 0;
			EPWM0->CMPDAT[3] = 0;
			EPWM0->CMPDAT[4] = 0;
		}
		else
		{
			vTaskSuspendAll();//���������
			EPWM0->CMPDAT[1] = limit_out(thr + out_pit + out_roll + out_yaw);//��ǰ
			EPWM0->CMPDAT[2] = limit_out(thr - out_pit + out_roll - out_yaw);//��ǰ
			EPWM0->CMPDAT[3] = limit_out(thr - out_pit - out_roll + out_yaw);//�Һ�
			EPWM0->CMPDAT[4] = limit_out(thr + out_pit - out_roll - out_yaw);//���
			xTaskResumeAll();//�ָ�������
		}
		delay_ms(2);
	}
}

void NRFProcess_task(void *pvParameters)
{
	u32 crc32,rxcrc32;//crc32:�����crc32	rxcrc32:���յ���crc32
	u8 i;
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRF_RX_Queue,Rx_BUF,portMAX_DELAY);//�Ӷ����ж�ȡ���յ�������
		CRC->CTL |= CRC_CTL_CRCEN_Msk;
		CRC_SET_SEED(0xFFFFFFFF);
		for(i=0;i<7;i++)
		{
			CRC->DAT = *(u32*)(Rx_BUF+i*4);
		}
		crc32 = CRC->CHECKSUM;
		rxcrc32 = *(u32*)(Rx_BUF+28);
		CRC->CTL &= ~CRC_CTL_CRCEN_Msk;
//		if(crc32 == rxcrc32)//crc32У����ȷ
//		{
			switch (Rx_BUF[1])
			{
				case 0x00://���յ�ң������
					memcpy(&RX_Data,Rx_BUF+2,sizeof(nrf_send));
					break;
				case 0x02:
					led2 = 1;
					if(Rx_BUF[2] == 0x01)//��ȡPID����
					{
						nrf_report_pid(&fram_data);
					}
					led2 = 0;
					break;
				case 0x01://COMMAND
					switch (Rx_BUF[2])
          {
          	case 0x01://ACCУ׼
							led2 = 1;
							fram_data.PITCH_ERR = pitch_raw;
							fram_data.ROLL_ERR = roll_raw;
							fram_data.YAW_ERR = yaw_raw;
						vTaskSuspendAll();//���������
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//����У׼ֵ
						xTaskResumeAll();//�ָ�������
							led2 = 0;
          		break;
          	case 0x02://GYROУ׼
							led2 = 1;
							fram_data.GYROX_ERR = gg[0];
							fram_data.GYROY_ERR = gg[1];
							fram_data.GYROZ_ERR = gg[2];
						vTaskSuspendAll();//���������
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//����У׼ֵ
						xTaskResumeAll();//�ָ�������
							led2 = 0;
          		break;
						case 0x03://MAGУ׼
          		break;
          	default:
          		break;
          }
					break;
				case 0x10://PID1-3
							led2 = 1;
							memcpy((u8*)&fram_data,Rx_BUF+2,18);
							led2 = 0;
          		break;
				case 0x11://PID4-6
							led2 = 1;
							memcpy(((u8*)&fram_data)+18,Rx_BUF+2,18);
						vTaskSuspendAll();//���������
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//����PID
						xTaskResumeAll();//�ָ�������
							PID_update();//����PID
							led2 = 0;
          		break;
				default:
					break;
			}
//		}
//		else
//		{
//			//CRC32������
//		}
	}
	
}

void NRF_TX_task(void *pvParameters)
{
	u8 Tx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRF_TX_Queue,Tx_BUF,portMAX_DELAY);//�Ӷ����ж�ȡҪ���͵�����
		vTaskSuspendAll();//���������
		NRF24L01_TxPacket(Tx_BUF);
		xTaskResumeAll();//�ָ�������
	}
}

void NRF_RX_task(void *pvParameters)
{
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	u8 sta;
	while(1)
	{
		vTaskSuspendAll();//���������
		sta = NRF24L01_Read_Reg(STATUSS);  //��ȡ״̬�Ĵ�����ֵ;
		if(sta & RX_OK) //���յ�����
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_BUF,RX_PLOAD_WIDTH);//��ȡ����
			xQueueSend(NRF_RX_Queue,Rx_BUF,portMAX_DELAY);//������з��ͽ��յ�����
		}
		if(sta & MAX_TX)
		{
			if(sta & 0x01)	//TX FIFO FULL
				NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUSS,sta); //���TX_DS��MAX_RT�жϱ�־
		xTaskResumeAll();//�ָ�������
		delay_ms(4);
	}
}

void SENDState_task(void *pvParameters)
{
	float EulerData[3];
	while(1)
	{
		xQueueReceive(EulerData_Queue,EulerData,portMAX_DELAY);//�Ӷ����ж�ȡ�ɻ���̬�Ƕ���Ϣ
		//�������ݵ����Ͷ���
		nrf_report_status((s16)(EulerData[0]*100.0f),(s16)(EulerData[1]*100.0f),(s16)(EulerData[2]*100.0f),0,0,!uav_lock);
		delay_ms(10);
		//�������ݵ����Ͷ���
		nrf_report_senser(accel_raw[0],accel_raw[1],accel_raw[2],gyro_raw[0],gyro_raw[1],gyro_raw[2],0,0,0);
		delay_ms(10);
		//�������ݵ����Ͷ���
		nrf_report_rcdata(RX_Data.thr,RX_Data.yaw,RX_Data.rol,RX_Data.pit,RX_Data.keyA*3000,RX_Data.keyB*3000,RX_Data.keyX*3000,RX_Data.keyY*3000,1500,1500);
		delay_ms(10);
		nrf_report_power((u16)(VBAT*2.0f*100.0f),0);
		delay_ms(10);
		portYIELD();//�������
	}
}


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //��������������
    xTaskCreate((TaskFunction_t )IMUupdate_task,     	
                (const char*    )"IMUupdate",   	
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,				
                (UBaseType_t    )6,	//�������ȼ�
                (TaskHandle_t*  )&IMUupdate_Handler);
    //����PID��̬��������
    xTaskCreate((TaskFunction_t )PIDControl_task,     
                (const char*    )"PIDControl_task",   
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,
                (UBaseType_t    )5,//�������ȼ�
                (TaskHandle_t*  )&PIDControl_Handler);   
		//�����������ݴ�������
    xTaskCreate((TaskFunction_t )NRFProcess_task,     
                (const char*    )"NRFProcess_task",   
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,
                (UBaseType_t    )5,//�������ȼ�
                (TaskHandle_t*  )&NRFProcess_Handler); 
		//�������߷�������
    xTaskCreate((TaskFunction_t )NRF_TX_task,     
                (const char*    )"NRF_TX_task",   
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,
                (UBaseType_t    )4,//�������ȼ�
                (TaskHandle_t*  )&NRF_TX_Handler); 
		//�������߽�������
    xTaskCreate((TaskFunction_t )NRF_RX_task,     
                (const char*    )"NRF_RX_task",   
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,
                (UBaseType_t    )4,//�������ȼ�
                (TaskHandle_t*  )&NRF_RX_Handler); 
		//����״̬����
    xTaskCreate((TaskFunction_t )SENDState_task,     
                (const char*    )"SENDState_task",   
                (uint16_t       )200, //�����ջ��С
                (void*          )NULL,
                (UBaseType_t    )3,//�������ȼ�
                (TaskHandle_t*  )&SENDState_Handler); 
								
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
		EulerData_Queue = xQueueCreate(1,12);//�����ɻ���̬�Ƕȶ���
		NRF_RX_Queue = xQueueCreate(1,32);//����NRF24L01�������ݶ���
		NRF_TX_Queue = xQueueCreate(2,32);//����NRF24L01�������ݶ���
		printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*��ӡʣ���ջ��С*/			
EPWM0_Init();								
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

int main()
 {
	Set_HCLK_192Mhz();
	NVIC_SetPriorityGrouping(4);
	delay_init();
	LED_Init();
	CLK_EnableModuleClock(CRC_MODULE);
	CRC_Open(CRC_32, 0, 0xFFFFFFFF, CRC_CPU_WDATA_32);
	uart_init(115200);
	FRAM_Init();
	memset(&fram_data,0,sizeof(fram_data));
	memset(&RX_Data,0,sizeof(RX_Data));
//	FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));
	FRAM_Read(0,(u8*)&fram_data,sizeof(fram_data));
	FRAM_Read(0,(u8*)&fram_data,sizeof(fram_data));
	NRF24L01_Init();
	NRF24L01_Mode(MODEL_TX2);//���ͣ�����գ�
	ADC_Init();
	Calculate_FilteringCoefficient(0.001f,10.f);
	MPU6050_Init();
		//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )200,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )1,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
	while(1);
}








void PID_update(void)//PID�����ı�����PID
{
	pidInit_t pid;
	pid.kp=(float)fram_data.PITPID_P * 0.1f;
	pid.ki=(float)fram_data.PITPID_I * 0.1f;
	pid.kd=(float)fram_data.PITPID_D * 0.1f;
	pidInit(&PITCH_PID, 0, pid, 0.001f);
	
	pid.kp=(float)fram_data.ROLPID_P * 0.1f;
	pid.ki=(float)fram_data.ROLPID_I * 0.1f;
	pid.kd=(float)fram_data.ROLPID_D * 0.1f;
	pidInit(&ROLL_PID, 0, pid, 0.001f);
	
	pid.kp=(float)fram_data.YAWPID_P * 0.1f;
	pid.ki=(float)fram_data.YAWPID_I * 0.1f;
	pid.kd=(float)fram_data.YAWPID_D * 0.1f;
	pidInit(&YAW_PID, 0, pid, 0.001f);
	
	pidReset(&PITCH_PID);
	pidReset(&ROLL_PID);
	pidReset(&YAW_PID);
}






void GPA_IRQHandler()		//NRF24L01 IRQ
{
	PA->INTSRC=PA->INTSRC;	
}

void TMR0_IRQHandler()		//TIMERx->INTSTS|=TIMER_INTSTS_TIF_Msk;
{
	TIMER0->INTSTS|=TIMER_INTSTS_TIF_Msk;
}





