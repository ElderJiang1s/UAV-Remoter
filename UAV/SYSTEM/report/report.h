#ifndef __REPORT_H
#define __REPORT_H
#include "cm4.h"

void niming_report(u8 fun,u8*data,u8 len);
void nrf_report(u8 fun,u8*data,u8 len);

void report_status(s16 roll,s16 pitch,s16 yaw,s32 alt,u8 mode,u8 armed);
void nrf_report_status(s16 roll,s16 pitch,s16 yaw,s32 alt,u8 mode,u8 armed);

void report_senser(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 magx,s16 magy,s16 magz);
void nrf_report_senser(s16 accx,s16 accy,s16 accz,s16 gyrox,s16 gyroy,s16 gyroz,s16 magx,s16 magy,s16 magz);

void report_rcdata(s16 thr,s16 yaw,s16 rol,s16 pit,s16 aux1,s16 aux2,s16 aux3,s16 aux4,s16 aux5,s16 aux6);
void nrf_report_rcdata(s16 thr,s16 yaw,s16 rol,s16 pit,s16 aux1,s16 aux2,s16 aux3,s16 aux4,s16 aux5,s16 aux6);

void report_power(u16 voltage,u16 current);
void nrf_report_power(u16 voltage,u16 current);

void report_check(u8 FREAM_HEAD,u8 CHECK_SUM);
void nrf_report_check(u8 FREAM_HEAD,u8 CHECK_SUM);

void report_pid(EEPROM_Data *pid);
void nrf_report_pid(EEPROM_Data *pid);

#endif

