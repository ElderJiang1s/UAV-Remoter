//***************************************************************************************
//�һ�΢�����������Դ���Ȩ���һ��Ŷ����У�δ���һ��Ŷ�ͬ�⣬�������������ϴ�����Դ�롣
//�뱾�������鼮<<���������DIY-����STM32΢������>>���ɱ�����������ʽ���棬���ݶԱ��װ�
//������������Լ�Ӳ����ض�������ϸ�Ľ��⣬����Ȥ�����ѿ��ԴӸ�����깺��
//�뱾������׵�Ӳ����http://fire-dragon.taobao.com
//������������˸���������ĸĽ�������ʱ�����Ǳ�����ϵ��
//QQ��16053729    �һ�QQȺ��234879071
//***************************************************************************************
#include "IMU.h"
#include "math.h"
#include "arm_math.h"

// ==================================================================================
// ����:
// ���붨��'halfT 'Ϊ���ڵ�һ�룬�Լ��˲����Ĳ���Kp��Ki
// ��Ԫ��'q0', 'q1', 'q2', 'q3'����Ϊȫ�ֱ���
// ��Ҫ��ÿһ���������ڵ���'IMUupdate()'����
// ���������ݵ�λ�ǻ���/�룬���ٶȼƵĵ�λ�޹���Ҫ����Ϊ�ᱻ�淶��
// ==================================================================================
#define Kp 	1.0f    // ��������
#define Ki 	0.001f  // ���ֳ���
#define halfT 0.0005f//������
#define T	0.001f  // ����Ϊ1ms
// ==================================================================================
// ��������
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     	// ��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;    	// �������ۼ�ֵ 

// ==================================================================================
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// ���ټ��㿪���ŵĵ���
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/******************************************************************************
����ԭ�ͣ�	void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
��    �ܣ�	���ٶ���ԭʼ����תΪ����
*******************************************************************************/ 
void Get_Radian(s16 *Gyro_in,float *Gyro_out)
{
	*Gyro_out = (float)(*Gyro_in * 0.0010653f);
	*(Gyro_out+1) = (float)(*(Gyro_in+1) * 0.0010653f);
	*(Gyro_out+2) = (float)(*(Gyro_in+2) * 0.0010653f);
}

// ==================================================================================
// ����ԭ�ͣ�void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
// ��        �ܣ������˲�������̬����
// ��        �룺���������ݼ����ٶȼ�����
// ==================================================================================

extern short gyro_raw[3], accel_raw[3];
extern EEPROM_Data fram_data;
extern float pitch_raw,roll_raw,yaw_raw;//δ����0�ĽǶ�
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* data) //data:pitch,roll,yaw
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
  //��Ԫ�����֣���õ�ǰ����̬
	float q0_last = q0;	
	float q1_last = q1;	
	float q2_last = q2;	
	float q3_last = q3;	

	//�Ѽ��ٶȼƵ���ά����ת�ɵ�λ����
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//�����������ٶȷ����ڷ���������ϵ�еı�ʾ��Ϊ��Ԫ����ʾ����ת����ĵ�����
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//���ٶȼƶ�ȡ�ķ������������ٶȷ���Ĳ�ֵ����������˼���
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//����ۻ���������ֳ������
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�ò���������PI����������ƫ�����������ݶ����е�ƫ����	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;


//	//һ�׽����㷨
	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
	q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
	q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
	q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT; 

	//���׽����㷨
//	float delta2 = (gx*gx + gy*gy + gz*gz)*T*T;
//	q0 = q0_last*(1-delta2/8) + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
//	q1 = q1_last*(1-delta2/8) + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
//	q2 = q2_last*(1-delta2/8) + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
//	q3 = q3_last*(1-delta2/8) + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT;

//	//���׽����㷨
//	float delta2 = (gx*gx + gy*gy + gz*gz)*T*T;
//	q0 = q0_last*(1-delta2/8) + (-q1_last*gx - q2_last*gy - q3_last*gz)*T*(0.5 - delta2/48);
//	q1 = q1_last*(1-delta2/8) + ( q0_last*gx + q2_last*gz - q3_last*gy)*T*(0.5 - delta2/48);
//	q2 = q2_last*(1-delta2/8) + ( q0_last*gy - q1_last*gz + q3_last*gx)*T*(0.5 - delta2/48);
//	q3 = q3_last*(1-delta2/8) + ( q0_last*gz + q1_last*gy - q2_last*gx)*T*(0.5 - delta2/48);

//	//�Ľ׽����㷨
//	float delta2 = (gx*gx + gy*gy + gz*gz)*T*T;
//	q0 = q0_last*(1 - delta2/8 + delta2*delta2/384) + (-q1_last*gx - q2_last*gy - q3_last*gz)*T*(0.5 - delta2/48);
//	q1 = q1_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gx + q2_last*gz - q3_last*gy)*T*(0.5 - delta2/48);
//	q2 = q2_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gy - q1_last*gz + q3_last*gx)*T*(0.5 - delta2/48);
//	q3 = q3_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gz + q1_last*gy - q2_last*gx)*T*(0.5 - delta2/48);
			
	//��Ԫ���淶��
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
	
	//��Ԫ��תŷ����
	roll_raw = atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3)*57.2957795f;
	pitch_raw  =  asin (2.0f*(q0*q2 - q1*q3))*57.2957795f;
	yaw_raw  +=  gyro_raw[2] * 0.0610351f * T;
	*data = pitch_raw - fram_data.PITCH_ERR;
	*(data+1) = roll_raw - fram_data.ROLL_ERR;
	*(data+2) = yaw_raw - fram_data.YAW_ERR;
}
