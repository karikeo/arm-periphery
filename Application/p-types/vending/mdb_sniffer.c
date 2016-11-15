#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "pt-vending.h"
#include "stdio.h"
#include "log.h"
#include "mdb.h"
#include "mdb_sniffer.h"

tMdbSniffer sniffer;
extern t_fb_desc fb;

#if defined MDB_SNIFFER

void sniffer_tx( u8* p, u16 n )
{
  u16 sz = sizeof(sniffer.tx_buf) - sniffer.tx_size;

  if ( n > sz )
  {
    n = sz;
  }
  
  if ( n > 0 )
  {
    memmove( &sniffer.tx_buf[ sniffer.tx_size ], p, n );
    sniffer.tx_size += n;  
    USART_ITConfig( USART3, USART_IT_TXE , ENABLE );      
  }
}

void sniffer_s_rx()
{  
  if ( (sniffer.m.raw.rx[ 0 ] & 0xF0) != 0x10 )
  {
      //return;
  }
  u8 n = 0;
  char buf[128];
#if 1
  float f = sniffer.s.raw.time;
  f /= 1000.0;
  n += sprintf(&buf[n],"[%07.1f] ", f);
#endif  
  buf[ n++ ] = SNIFFER_SLAVE_CH;
  buf[ n++ ] = ':';  
  for (u8 i = 0; i < sniffer.s.raw.rx_size; i++ )
  {
    n += sprintf(&buf[n],"%.2X ", sniffer.s.raw.rx[i]);
  }  
  buf[ n-1 ] = 0x0D;
  
  sniffer_tx( buf, n );
  memset(sniffer.s.raw.rx, 0, sizeof(sniffer.s.raw.rx));
}

void sniffer_m_rx()
{
  if ( (sniffer.m.raw.rx[ 0 ] & 0xF0) != 0x10 )
  {
      //return;
  }
  
  u8 n = 0;
  char buf[128];
#if 1 
  float f = sniffer.m.raw.time;
  f /= 1000.0;
  n += sprintf(&buf[n],"[%07.1f] ", f);
#endif  
  buf[ n++ ] = SNIFFER_MASTER_CH;
  buf[ n++ ] = ':';  
  for (u8 i = 0; i < sniffer.m.raw.rx_size; i++ )
  {
    n += sprintf(&buf[n],"%.2X ", sniffer.m.raw.rx[i]);
  }  
  buf[ n-1 ] = '\r';

  sniffer_tx( buf, n );  
  memset(sniffer.m.raw.rx, 0, sizeof(sniffer.m.raw.rx));
}

void sniffer_rx_msg()
{  
  s16 sz = 0;  
  u8* rx = sniffer.rx_buf;
  u16 rx_size = sniffer.rx_size;
  void (*tx)(u16*,u16) = 0;
  
  log( "sniffer rx:%d ", rx_size );
  for ( int i = 0 ; i < rx_size; i++ )
  {
    log( "%c",rx[i] );    
  }
  
  if ( rx_size > 2 && rx[ 1 ] == ':' )
  {    
    sz = rx_size - 2;
    if ( rx[ 0 ] == SNIFFER_MASTER_CH )
    {   
      tx = fb_m_tx;
    }
    else if ( rx[ 0 ] == SNIFFER_SLAVE_CH )
    {      
      tx = fb_s_tx;
    }    
    
    if ( sz > 0 && tx )
    {
      tx( (u16*)&rx[2], sz );
    }
  }
  else if ( rx_size == 2 && rx[ 0 ] == 'd' && rx[ 1 ] == '\r')
  {    
    fb.cashless.display_tr = 1;
  }  
  else if ( rx_size == 2 && rx[ 0 ] == 'b' && rx[ 1 ] == '\r')
  {    
    fb.cashless.begin_session_tr = 1;    
  }
  else if ( rx_size == 2 && rx[ 0 ] == 'c' && rx[ 1 ] == '\r')
  {    
    fb.cashless.cancel_session_tr = 1;    
  }  
  else
  {
    log("sniffer rx incorrect format\r\n");
  }
  memset(sniffer.rx_buf, 0, sizeof(sniffer.rx_buf));
  sniffer.rx_size = 0;
}

void sniffer_isr( void* uart )
{
  if ( USART_GetFlagStatus( uart, USART_FLAG_RXNE ) ) 
  {
    // Push a new data into the receiver buffer
    volatile u8 b = USART_ReceiveData(uart);
    if ( sniffer.rx_size < sizeof(sniffer.rx_buf) ) 
    {
       sniffer.rx_buf[ sniffer.rx_size++ ] = b;
    }
    OS_RetriggerTimer( &sniffer.timer );        
  }

  if ( USART_GetFlagStatus( uart, USART_FLAG_TXE ) && ( USART_GetITStatus( uart, USART_IT_TXE ) == SET ) ) 
  {
    if( sniffer.tx_size > 0 ) 
    {
      volatile u8 b =  sniffer.tx_buf[ sniffer.tx_cnt++ ];
      USART_SendData( uart, b );
      sniffer.tx_size--;
    }
    else if( sniffer.tx_size == 0 ) 
    {
      sniffer.tx_cnt = 0;
      USART_ITConfig( uart, USART_IT_TXE , DISABLE );         
    }  
  }  
}

#endif
