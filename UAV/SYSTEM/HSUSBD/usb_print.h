#ifndef __USB_PRINT_H
#define __USB_PRINT_H
#include "cm4.h"
#include "vcom_serial.h"


void VCOM_TransferData(void);

int fputc(int ch, FILE *stream);

#endif

