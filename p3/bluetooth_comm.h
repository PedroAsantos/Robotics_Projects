// bluetooth_comm.h
// DETI-UA
// JLA, 06/11/2014
//
#ifndef __BLUETOOTH_COMM_H
#define __BLUETOOTH_COMM_H

#include <detpic32.h>

extern int bt_enable;

#ifdef printf
	#undef printf
#endif

#define printf(format...) ({if(!bt_enable) \
			xprintf((void (*)(int,char))NULL, ## format);\
		else\
			xprintf(txc_bt, ## format);\
		})

#define bt_on() {bt_enable=true;}
#define bt_off() {bt_enable=false;}

void configBTUart(int channel, unsigned int baudrate);
void txc_bt(int, char);
#endif

