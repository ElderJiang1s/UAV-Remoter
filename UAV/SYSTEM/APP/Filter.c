#include "math.h"
#include "Filter.h"


static float ACC_IIR_FACTOR;
/******************************************************************************
����ԭ�ͣ�	void Calculate_FilteringCoefficient(float Time, float Cut_Off)
��    �ܣ�	iir��ͨ�˲���������
*******************************************************************************/ 
void Calculate_FilteringCoefficient(float Time, float Cut_Off)
{
	ACC_IIR_FACTOR = Time /( Time + 1/(2.0f*3.1415927f*Cut_Off) );
}

/******************************************************************************
����ԭ�ͣ�	void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out)
��    �ܣ�	iir��ͨ�˲�
*******************************************************************************/ 
void ACC_IIR_Filter(s16 *Acc_in,s16 *Acc_out)
{
	*Acc_out = *Acc_out + ACC_IIR_FACTOR*(*Acc_in - *Acc_out); 
	*(Acc_out+1) = *(Acc_out+1) + ACC_IIR_FACTOR*(*(Acc_in+1) - *(Acc_out+1)); 
	*(Acc_out+2) = *(Acc_out+2) + ACC_IIR_FACTOR*(*(Acc_in+2) - *(Acc_out+2)); 
}

#define Filter_Num 2
/******************************************************************************
����ԭ�ͣ�	void Gyro_Filter(struct _gyro *Gyro_in,struct _gyro *Gyro_out)
��    �ܣ�	gyro���ڻ����˲�
*******************************************************************************/ 
void Gyro_Filter(s16 *Gyro_in,s16 *Gyro_out)
{
	static int16_t Filter_x[Filter_Num],Filter_y[Filter_Num],Filter_z[Filter_Num];
	static uint8_t Filter_count;
	int32_t Filter_sum_x=0,Filter_sum_y=0,Filter_sum_z=0;
	uint8_t i=0;
	
	Filter_x[Filter_count] = *Gyro_in;
	Filter_y[Filter_count] = *(Gyro_in+1);
	Filter_z[Filter_count] = *(Gyro_in+2);

	for(i=0;i<Filter_Num;i++)
	{
		Filter_sum_x += Filter_x[i];
		Filter_sum_y += Filter_y[i];
		Filter_sum_z += Filter_z[i];
	}	
	
	*Gyro_out = Filter_sum_x / Filter_Num;
	*(Gyro_out+1) = Filter_sum_y / Filter_Num;
	*(Gyro_out+2) = Filter_sum_z / Filter_Num;
	
	Filter_count++;
	if(Filter_count == Filter_Num)
		Filter_count=0;
}


