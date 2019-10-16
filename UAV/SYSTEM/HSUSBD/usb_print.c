#include "usb_print.h"


/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

volatile uint8_t comRbuf[RXBUFSIZE] __attribute__((aligned(4)));
volatile uint8_t comTbuf[TXBUFSIZE]__attribute__((aligned(4)));
uint8_t gRxBuf[64] __attribute__((aligned(4))) = {0};
uint8_t gUsbRxBuf[64] __attribute__((aligned(4))) = {0};




void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check if any data to send to USB & USB is ready to send them out */
    if(comRbytes && (gu32TxSize == 0))
    {
        i32Len = comRbytes;
        if(i32Len > EPA_MAX_PKT_SIZE)
            i32Len = EPA_MAX_PKT_SIZE;

        for(i=0; i<i32Len; i++)
        {
            gRxBuf[i] = comRbuf[comRhead++];
            if(comRhead >= RXBUFSIZE)
                comRhead = 0;
        }

        NVIC_DisableIRQ(TMR0_IRQn);
        comRbytes -= i32Len;
        NVIC_EnableIRQ(TMR0_IRQn);

        gu32TxSize = i32Len;
        for (i=0; i<i32Len; i++)
            HSUSBD->EP[EPA].EPDAT_BYTE = gRxBuf[i];
        HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
        HSUSBD->EP[EPA].EPTXCNT = i32Len;
        HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_INTKIEN_Msk);
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
    {
        for(i=0; i<gu32RxSize; i++)
        {
            comTbuf[comTtail++] = gUsbRxBuf[i];
            if(comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        NVIC_DisableIRQ(TMR0_IRQn);
        comTbytes += gu32RxSize;
        NVIC_EnableIRQ(TMR0_IRQn);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */
    }

    /* Process the software Tx FIFO */
    if(comTbytes)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART_WRITE(UART0, comTbuf[comThead++]);
            if(comThead >= TXBUFSIZE)
                comThead = 0;

            NVIC_DisableIRQ(TMR0_IRQn);
            comTbytes--;
            NVIC_EnableIRQ(TMR0_IRQn);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}
extern u8 uart_rxdata[32];
extern void RX_Process(u8 fun,u8* data,u8 len);

void VCOM_Handler(u8 dat)
{
	static u8 aa=0,af=0,fun=0,len=0,cnt=0,sum=0;
	
	if(dat==0xAA&&aa==0)
	{
		aa=1;
		uart_rxdata[0]=dat;
	}else if(dat==0xAF&&aa==1&&af==0 )
	{
		af=1;
		uart_rxdata[1]=dat;
	}else if(aa==1&&af==1&&fun==0)
	{
		fun=dat;
		uart_rxdata[2]=fun;
	}else if(aa==1&&af==1&&fun!=0&&len==0)
	{
		len=dat;
		uart_rxdata[3]=len;
	}else if(aa==1&&af==1&&fun!=0&&len!=0&&cnt!=len)
	{
		uart_rxdata[4+cnt]=dat;
		cnt++;
	}else if(aa==1&&af==1&&fun!=0&&len!=0&&cnt==len)
	{
		sum=dat;
		uart_rxdata[4+cnt]=sum;
		RX_Process(fun,uart_rxdata,len+4);//len no include sum
		aa=0;
		af=0;
		fun=0;
		len=0;
		cnt=0;
		sum=0;
	}
}

void isDataRDY(void)//检查vcom是否收到数据，需要不断运行
{
	uint8_t bInChar;
	int32_t size;
	if(comTbytes)
        {
            /* Fill the Tx FIFO */
            size = comTbytes;
            if(size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while(size)
            {
                bInChar = comTbuf[comThead++];
                //UART_WRITE(UART0, bInChar);
							VCOM_Handler(bInChar);
                if(comThead >= TXBUFSIZE)
                    comThead = 0;
                comTbytes--;
                size--;
            }
        }
}
