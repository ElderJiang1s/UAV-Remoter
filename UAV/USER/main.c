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


TaskHandle_t StartTask_Handler,IMUupdate_Handler,PIDControl_Handler,NRFProcess_Handler,NRF_TX_Handler,NRF_RX_Handler,SENDState_Handler;//任务句柄
QueueHandle_t EulerData_Queue;//飞机姿态角度队列
QueueHandle_t NRF_RX_Queue;//NRF24L01接收数据队列
QueueHandle_t NRF_TX_Queue;//NRF24L01发送数据队列


EEPROM_Data fram_data;
nrf_send RX_Data;
float set_pit = 0,set_roll = 0,set_yaw = 0;//要设置的角度 受遥控器控制
s16 aa[3],gg[3];	//未滤波的accel gyro
short gyro_raw[3], accel_raw[3];//滤波后的accel gyro
float gyro[3];		//弧度
float pitch_raw,roll_raw,yaw_raw;//未经调0的角度
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
		vTaskDelayUntil(&lastWakeTime, 1);		/*1ms周期延时*/
		vTaskSuspendAll();//挂起调度器
		MPU_Get_Accelerometer(aa,aa+1,aa+2);
		MPU_Get_Gyroscope(gg,gg+1,gg+2);
		VBAT = (float)ADC_GetData(8) * 0.000805664f / 1.0518f;
		xTaskResumeAll();//恢复调度器
		ACC_IIR_Filter(aa,accel_raw);//accel滤波
		Gyro_Filter(gg,gyro_raw);//gyro滤波
		gyro_raw[0] -= fram_data.GYROX_ERR;
		gyro_raw[1] -= fram_data.GYROY_ERR;
		gyro_raw[2] -= fram_data.GYROZ_ERR;//gyro减去误差
		Get_Radian(gyro_raw,gyro);//角速度数据转为弧度
		IMUupdate(gyro[0],gyro[1],gyro[2],accel_raw[0],accel_raw[1],accel_raw[2],EulerData);//姿态解算
		xQueueOverwrite(EulerData_Queue,EulerData);//向队列中发送飞机姿态角度信息带覆写
		//vTaskDelay(1);
	}
}

void PIDControl_task(void *pvParameters)
{
	float out_pit,out_roll,out_yaw;
	float EulerData[3];
	float thr;//油门
	PID_update();//更新PID参数
	while(1)
	{
		xQueueReceive(EulerData_Queue,EulerData,portMAX_DELAY);//从队列中读取飞机姿态角度信息
		thr = (float)RX_Data.thr;
		set_roll = mapf(3000.0f - ((float)RX_Data.pit),0,3000.0f,-10.0f,10.0f);
		set_pit = mapf((float)RX_Data.rol,0,3000.0f,-10.0f,10.0f);
		set_yaw = mapf((float)RX_Data.yaw,0,3000.0f,-10.0f,10.0f);//根据遥控器数据设置飞行方向
		out_pit = pidUpdate(&PITCH_PID,set_pit - EulerData[0]);
		out_roll = pidUpdate(&ROLL_PID,set_roll - EulerData[1]);
		out_yaw = pidUpdate(&YAW_PID,set_yaw - EulerData[2]);//PID计算
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
			vTaskSuspendAll();//挂起调度器
			EPWM0->CMPDAT[1] = limit_out(thr + out_pit + out_roll + out_yaw);//左前
			EPWM0->CMPDAT[2] = limit_out(thr - out_pit + out_roll - out_yaw);//右前
			EPWM0->CMPDAT[3] = limit_out(thr - out_pit - out_roll + out_yaw);//右后
			EPWM0->CMPDAT[4] = limit_out(thr + out_pit - out_roll - out_yaw);//左后
			xTaskResumeAll();//恢复调度器
		}
		delay_ms(2);
	}
}

void NRFProcess_task(void *pvParameters)
{
	u32 crc32,rxcrc32;//crc32:计算的crc32	rxcrc32:接收到的crc32
	u8 i;
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRF_RX_Queue,Rx_BUF,portMAX_DELAY);//从队列中读取接收到的数据
		CRC->CTL |= CRC_CTL_CRCEN_Msk;
		CRC_SET_SEED(0xFFFFFFFF);
		for(i=0;i<7;i++)
		{
			CRC->DAT = *(u32*)(Rx_BUF+i*4);
		}
		crc32 = CRC->CHECKSUM;
		rxcrc32 = *(u32*)(Rx_BUF+28);
		CRC->CTL &= ~CRC_CTL_CRCEN_Msk;
//		if(crc32 == rxcrc32)//crc32校验正确
//		{
			switch (Rx_BUF[1])
			{
				case 0x00://接收到遥控数据
					memcpy(&RX_Data,Rx_BUF+2,sizeof(nrf_send));
					break;
				case 0x02:
					led2 = 1;
					if(Rx_BUF[2] == 0x01)//读取PID请求
					{
						nrf_report_pid(&fram_data);
					}
					led2 = 0;
					break;
				case 0x01://COMMAND
					switch (Rx_BUF[2])
          {
          	case 0x01://ACC校准
							led2 = 1;
							fram_data.PITCH_ERR = pitch_raw;
							fram_data.ROLL_ERR = roll_raw;
							fram_data.YAW_ERR = yaw_raw;
						vTaskSuspendAll();//挂起调度器
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//保存校准值
						xTaskResumeAll();//恢复调度器
							led2 = 0;
          		break;
          	case 0x02://GYRO校准
							led2 = 1;
							fram_data.GYROX_ERR = gg[0];
							fram_data.GYROY_ERR = gg[1];
							fram_data.GYROZ_ERR = gg[2];
						vTaskSuspendAll();//挂起调度器
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//保存校准值
						xTaskResumeAll();//恢复调度器
							led2 = 0;
          		break;
						case 0x03://MAG校准
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
						vTaskSuspendAll();//挂起调度器
							FRAM_Write(0,(u8*)&fram_data,sizeof(fram_data));//保存PID
						xTaskResumeAll();//恢复调度器
							PID_update();//更新PID
							led2 = 0;
          		break;
				default:
					break;
			}
//		}
//		else
//		{
//			//CRC32错误处理
//		}
	}
	
}

void NRF_TX_task(void *pvParameters)
{
	u8 Tx_BUF[32]	__attribute__((aligned (4)));
	while(1)
	{
		xQueueReceive(NRF_TX_Queue,Tx_BUF,portMAX_DELAY);//从队列中读取要发送的数据
		vTaskSuspendAll();//挂起调度器
		NRF24L01_TxPacket(Tx_BUF);
		xTaskResumeAll();//恢复调度器
	}
}

void NRF_RX_task(void *pvParameters)
{
	u8 Rx_BUF[32]	__attribute__((aligned (4)));
	u8 sta;
	while(1)
	{
		vTaskSuspendAll();//挂起调度器
		sta = NRF24L01_Read_Reg(STATUSS);  //读取状态寄存器的值;
		if(sta & RX_OK) //接收到数据
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,Rx_BUF,RX_PLOAD_WIDTH);//读取数据
			xQueueSend(NRF_RX_Queue,Rx_BUF,portMAX_DELAY);//向队列中发送接收到数据
		}
		if(sta & MAX_TX)
		{
			if(sta & 0x01)	//TX FIFO FULL
				NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUSS,sta); //清除TX_DS或MAX_RT中断标志
		xTaskResumeAll();//恢复调度器
		delay_ms(4);
	}
}

void SENDState_task(void *pvParameters)
{
	float EulerData[3];
	while(1)
	{
		xQueueReceive(EulerData_Queue,EulerData,portMAX_DELAY);//从队列中读取飞机姿态角度信息
		//发送数据到发送队列
		nrf_report_status((s16)(EulerData[0]*100.0f),(s16)(EulerData[1]*100.0f),(s16)(EulerData[2]*100.0f),0,0,!uav_lock);
		delay_ms(10);
		//发送数据到发送队列
		nrf_report_senser(accel_raw[0],accel_raw[1],accel_raw[2],gyro_raw[0],gyro_raw[1],gyro_raw[2],0,0,0);
		delay_ms(10);
		//发送数据到发送队列
		nrf_report_rcdata(RX_Data.thr,RX_Data.yaw,RX_Data.rol,RX_Data.pit,RX_Data.keyA*3000,RX_Data.keyB*3000,RX_Data.keyX*3000,RX_Data.keyY*3000,1500,1500);
		delay_ms(10);
		nrf_report_power((u16)(VBAT*2.0f*100.0f),0);
		delay_ms(10);
		portYIELD();//任务调度
	}
}


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建传感器任务
    xTaskCreate((TaskFunction_t )IMUupdate_task,     	
                (const char*    )"IMUupdate",   	
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,				
                (UBaseType_t    )6,	//任务优先级
                (TaskHandle_t*  )&IMUupdate_Handler);
    //创建PID姿态控制任务
    xTaskCreate((TaskFunction_t )PIDControl_task,     
                (const char*    )"PIDControl_task",   
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,
                (UBaseType_t    )5,//任务优先级
                (TaskHandle_t*  )&PIDControl_Handler);   
		//创建无线数据处理任务
    xTaskCreate((TaskFunction_t )NRFProcess_task,     
                (const char*    )"NRFProcess_task",   
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,
                (UBaseType_t    )5,//任务优先级
                (TaskHandle_t*  )&NRFProcess_Handler); 
		//创建无线发送任务
    xTaskCreate((TaskFunction_t )NRF_TX_task,     
                (const char*    )"NRF_TX_task",   
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,
                (UBaseType_t    )4,//任务优先级
                (TaskHandle_t*  )&NRF_TX_Handler); 
		//创建无线接收任务
    xTaskCreate((TaskFunction_t )NRF_RX_task,     
                (const char*    )"NRF_RX_task",   
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,
                (UBaseType_t    )4,//任务优先级
                (TaskHandle_t*  )&NRF_RX_Handler); 
		//发送状态任务
    xTaskCreate((TaskFunction_t )SENDState_task,     
                (const char*    )"SENDState_task",   
                (uint16_t       )200, //任务堆栈大小
                (void*          )NULL,
                (UBaseType_t    )3,//任务优先级
                (TaskHandle_t*  )&SENDState_Handler); 
								
    vTaskDelete(StartTask_Handler); //删除开始任务
		EulerData_Queue = xQueueCreate(1,12);//创建飞机姿态角度队列
		NRF_RX_Queue = xQueueCreate(1,32);//创建NRF24L01接收数据队列
		NRF_TX_Queue = xQueueCreate(2,32);//创建NRF24L01发送数据队列
		printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/			
EPWM0_Init();								
    taskEXIT_CRITICAL();            //退出临界区
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
	NRF24L01_Mode(MODEL_TX2);//发送（兼接收）
	ADC_Init();
	Calculate_FilteringCoefficient(0.001f,10.f);
	MPU6050_Init();
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








void PID_update(void)//PID参数改变后更新PID
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





