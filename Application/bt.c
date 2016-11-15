#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
//#include "vending/pt-vending.h"
//#include "dex.h"
#include "vending/mdb.h"
#include "stdio.h"
#include "log.h"
#if defined LCD
#include "lcd_2x16.h"
#endif
#include "spi_comm.h"
//#include "mdb_sniffer.h"
#include "sw_uart.h"
#include "bt.h"

extern t_sw_uart sw_uart;
extern t_fb_desc fb;
//extern OS_TASK TCB2;

#define BT_CMD_BALANCE "BALANCE"

void bt_rx_msg_handler(void)
{  
   const char* p = (const char*) sw_uart.rx.buf;
   int d;
   if ( sscanf( p,"BALANCE=%d", &d ) > 0 )
   {
      fb.cashless.funds_available = d;
      fb.cashless.begin_session_tr = 1; 
   }
   else if ( strcmp( p,"CANCEL\r" ) == 0 )
   {
      fb.cashless.cancel_session_tr = 1;
   }   
   sw_uart_reset_buffers();
}
