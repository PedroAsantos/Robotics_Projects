***************************************************************************
Instructions to use bluetooth communications in the DETI Robot
***************************************************************************

1) In your main module:
 1.1) include the "bluetooth_comm.h" file

    #include "bluetooth_comm.h"

 1.2) just after the initPIC32() instruction add the following:

    configBTUart(3, 115200); // Configure Bluetooth UART
    bt_on();     // enable bluetooth channel; printf
                // is now redirected to the bluetooth UART

 1.3) to disable the bluetooth channel (default is disabled)
    bt_off();    // printf will send data to the USB port (default)

2) To send data to the bluetooth port use the usual printf() command

3) Compilation
    pcompile robfile.c mr32.c bluetooth_comm.c

4) To receive data in your PC
 4.1) Open a terminal and:
    * type ./bt.sh
    * wait until the following message appears:

      Connected /dev/rfcomm0 to 00:06:66:45:DC:6B on channel 1
      Press CTRL-C for hangup

 4.2) Open another terminal and type:
      
      pterm -p /dev/rfcomm0

(any problem should be reported to jla@ua.pt)
***************************************************************************

