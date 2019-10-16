#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

#define SYSTEM_SUPPORT_OS	 1
static u32 sysTickCnt=0;
u32 fac_us;
#if SYSTEM_SUPPORT_OS		
    static u16 fac_ms=0;				        //ms��ʱ������,��os��,����ÿ�����ĵ�ms��
		extern void xPortSysTickHandler(void);
#endif
#if SYSTEM_SUPPORT_OS		
void SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
  {
       xPortSysTickHandler();	
  }
	else
	{
		sysTickCnt++;
	}
}

#endif


void delay_init(void)
{
#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
	u32 reload;
#endif
	SysTick->CTRL=0x00;
	SYS_UnlockReg();
	CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HXT);	//systickʱ��ԴΪHXT
	SYS_LockReg();
	fac_us=12;			//HXT=12MHz	����12��=1us
#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
	reload=12;					    //ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/configTICK_RATE_HZ;	//����delay_ostickspersec�趨���ʱ��
											//reloadΪ24λ�Ĵ���,���ֵ:16777216,��180M��,Լ��0.745s����	
	fac_ms=1000/configTICK_RATE_HZ;		//����OS������ʱ�����ٵ�λ	   
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//����SYSTICK�ж�
	SysTick->LOAD=reload; 					//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //����SYSTICK
#endif
}


#if SYSTEM_SUPPORT_OS 						//�����Ҫ֧��OS.
//��ʱnus
//nus:Ҫ��ʱ��us��.	
//nus:0~204522252(���ֵ��2^32/fac_us@fac_us=168)	    								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};										    
}  
//��ʱnms
//nms:Ҫ��ʱ��ms��
//nms:0~65535
void delay_ms(u32 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
	{		
		if(nms>=fac_ms)						//��ʱ��ʱ�����OS������ʱ������ 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS��ʱ
		}
		nms%=fac_ms;						//OS�Ѿ��޷��ṩ��ôС����ʱ��,������ͨ��ʽ��ʱ    
	}
	delay_us((u32)(nms*1000));				//��ͨ��ʽ��ʱ
}

//��ʱnms,���������������
//nms:Ҫ��ʱ��ms��
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

/********************************************************
*getSysTickCnt()
*���ȿ���֮ǰ ���� sysTickCnt
*���ȿ���֮ǰ ���� xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*ϵͳ�Ѿ�����*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}




#else  //����ucosʱ

void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us;
	SysTick->VAL=0x00;        								//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;		//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//�ȴ�ʱ�䵽�� 
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      									//��ռ�����	
}

void delay_ms(u32 nms)											//���0xFFFFFF/1000/12=1398.10125ms
{
	u32 temp;
	SysTick->LOAD=nms*fac_us*1000;
	SysTick->VAL=0x00;        								//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;		//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//�ȴ�ʱ�䵽�� 
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      									//��ռ�����	
}

#endif

