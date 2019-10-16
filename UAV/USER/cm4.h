#ifndef __CM_4
#define __CM_4
#include "NuMicro.h"

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


/************************ CLK *****************************/
void Set_HCLK_192Mhz(void);


/************************ GPIO ****************************/
void LED_Init(void);
#define led1 PB15
#define led2 PB14
void KEY_Init(void);


/************************ TIMER	***************************/
void TIM0_IT_Init(u32 nus);
void EPWM0_Init(void);
void EPWM_SetDutyCycle(EPWM_T *epwm,u32 duty,u32 nch);

/************************ ADC *****************************/
void ADC_Init(void);
u32 ADC_GetData(u32 nch);
float ADC_GetTemp(void);


/************************ DAC *****************************/
void DAC_Init(void);
void DAC_Output(DAC_T *dac,u32 val);





/************************ RTC *****************************/
void LXT_Init(void);



/************************ DMA *****************************/
void DMA_Init(u32 nch,u32 Width,u32 TransCount,u32 Peripheral,u32 SrcAddr,u32 DirAddr,u32 SrcCtrl,u32 DstCtrl);



/************************	USART ***************************/
void uart_init(u32 bound);


float mapf(float val, float I_Min, float I_Max, float O_Min, float O_Max);



/************************	OTHER ***************************/
void INTX_DISABLE(void);
void INTX_ENABLE(void);



#endif
