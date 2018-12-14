// bluetooth_comm.c
//
// DETI-UA
// JLA, 06/11/2014
//
#include "bluetooth_comm.h"

int bt_enable = 0;	// false

#define UART1_ADDRESS	U1MODE
int uart_offsets[] = {0x0000, 0x0800, 0x0400, 0x0200, 0x0A00, 0x0600}; 

static int uartChannel = 3;

// ****************************************************************************
// default settings assumed:
//  - baudrate factor = 16 (if the core frequency is 10 MHz, then the BRGH 
//  						factor should be 4)
//  - 1 stop bit
//  - parity N
//  - Auto Baud = OFF
void configBTUart(int channel, unsigned int baudrate)
{
	int *umode = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x00);
	int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);
	int *ubrg = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x40);

	uartChannel = channel;

	if(baudrate < 300 || baudrate > 115200)
		baudrate = 115200;

	*ubrg = (PBCLK + 8 * baudrate) / (16 * baudrate) - 1;

	*usta = *usta | 0x1400;		// Enable receiver section
								// Enable transmiter sction
	*umode = *umode | 0x8000;	// UxMODEbits.ON = 1; 	// UART ON
}


// ****************************************************************************
void sendChar(int channel, char txChar)
{
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);
	int *utxreg = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x20);

	while( (*usta & 0x200) != 0);
	*utxreg = txChar;
}


void txc_bt(int n, char c)
{
	sendChar(uartChannel, c);
}

