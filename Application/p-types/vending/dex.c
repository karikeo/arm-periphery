#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "pt-vending.h"
#include "stdio.h"
#include "log.h"
#if defined LCD
  #include "lcd_2x16.h"
#endif
#include "dex_sample.c"
#include "spi_comm.h"

#include "dex.h"

#if defined DEX

#define TX USART_ITConfig( DEX, USART_IT_TXE , ENABLE )

tDex dex;
char* const dexStateName[] = 
{
  "Initial",
  "Idle",
  "SendENQ",
  "CheckENQ",
  "SlaveHandshake1",
  "SlaveHandshake2",
  "SlaveHandshake3",
  "MasterHandshake1",
  "MasterHandshake2",
  "ThirdHandshake",
  "SendAuditData",
  "RecvAuditData1",
  "RecvAuditData2",
  "RecvAuditData3"
};

void dex_init()
{
   dex.state = dexState_Idle;
   memset( dex.rx_buf, 0, sizeof( dex.rx_buf ) );
   dex.rx_size = 0;
}

int read_dex( void ) {

   if ( dex.sema ) {
      log ( "dex sema\r\n" );
      return 0;
   }

   dex.sema = 1;

   log( ">read_dex(), st:%d-%s\r\n", dex.state, dexStateName[ dex.state ] );
   
   
   if ( !dex.enable ) {
      dex.state = dexState_Idle;
   }

   switch ( dex.state ) {

      case dexState_Initial:
         OS_StopTimer( &dex.timer );
         dex.bMasterHSDone = 0;
         dex.bSlaveHSDone = 0;
         dex.audit_data_valid = 0;

         dex.result = dexResult_InProgress;
         strcpy( dex.resultDescr, "" );

         memset( dex.rx_buf, 0 , sizeof( dex.rx_buf ) );
         dex.rx_size = 0;

         dex.bDleToggle = 0;

         dex.triggers.bits.query_cmplt = 0;

         //gotoDexState( dexState_SendENQ, 2000 );
         gotoDexState( dexState_CheckENQ, 2000 );
         break;

      case dexState_SendENQ:
         dex.tx_size = 0;
         dex.tx_buf[ dex.tx_size++ ] = ENQ;
         TX;

         //dex.rx_size = 0;
         gotoDexState( dexState_CheckENQ, 2000 );
         break;

      case dexState_CheckENQ:
         if ( dex.rx_size == 0 ) {
            log(" no answer for ENQ\r\n");
            gotoDexState( dexState_SendENQ, 2000 );
            break;
         }

         //log(" CheckENQ: received %d bytes:\r\n", dex.rx_size);
         printDexBytes( " rx", dex.rx_buf, dex.rx_size );

         if ( dex.rx_buf[ 0 ] == ENQ )  {
            dex.rx_size = 0;
            gotoDexState( dexState_SlaveHandshake1, 2000 );
            break;
         }

         if ( dex.rx_buf[ 0 ] == DLE )  {
            dex.rx_size = 0;
            gotoDexState( dexState_MasterHandshake1, 200 );
            break;
         }

         log(" CheckENQ: unexpected bytes\r\n");
         dex.rx_size = 0;
         gotoDexState( dexState_CheckENQ, 1000 );
         break;

      case dexState_MasterHandshake1:
         dex.tx_size = 0;
         dex.tx_buf[ dex.tx_size++ ] = DLE;
         dex.tx_buf[ dex.tx_size++ ] = SOH;

         if ( dex.bSlaveHSDone )
            dex.tx_size += sprintf( (char*) &dex.tx_buf[ dex.tx_size ], "001234567890RR01L01");
         else
            dex.tx_size += sprintf( (char*) &dex.tx_buf[ dex.tx_size ], "1234567890RR01L01");

         dex.tx_buf[ dex.tx_size++ ] = DLE;
         dex.tx_buf[ dex.tx_size++ ] = ETX;

         dex.crc = 0;
         calc_crc2( (char*) dex.tx_buf, dex.tx_size );

         dex.tx_buf[ dex.tx_size++ ] = ( dex.crc & 0xFF);
         dex.tx_buf[ dex.tx_size++ ] = ( dex.crc >> 8 ) & 0xFF;
         TX;

#if defined ( DEX_SAMPLE )
         strcpy( (char*)dex.audit_data, dex_sample );
         dex.audit_size = strlen( (const char*)dex.audit_data );
         dex.audit_data_valid = 1;
         dex.triggers.bits.query_cmplt = 1;
         gotoDexState( dexState_Idle, 1000 );
#else
         gotoDexState( dexState_MasterHandshake2, 1000 );
#endif
         break;

      case dexState_MasterHandshake2:
         if ( !dex.rx_size ) {
            dex.bMasterHSDone = 0;
            dex.bSlaveHSDone = 0;
            gotoDexState( dexState_SendENQ, 1000 );
            break;
         }

         printDexBytes( " rx", dex.rx_buf, dex.rx_size );

         if ( dex.rx_size == 2 ) {
            if ( dex.rx_buf[ 0 ] == DLE && ( dex.rx_buf[ 1 ] == '1' || dex.rx_buf[ 1 ] == '0') ) {
               dex.tx_size = 0;
               dex.tx_buf[ dex.tx_size++ ] = EOT;
               TX;

               dex.bMasterHSDone = 1;
               if ( dex.bSlaveHSDone ) {
                  dex.audit_size = 0;
                  gotoDexState( dexState_RecvAuditData1, 1000 );
               } else {
                  gotoDexState( dexState_SlaveHandshake1, 1000 );
               }
               dex.rx_size = 0;
               break;
            }
         }

         dex.rx_size = 0;
         gotoDexState( dexState_MasterHandshake2, 1000 );
         break;

      case dexState_SlaveHandshake1:
#if 1
         if ( !dex.rx_size ) {
            dex.bMasterHSDone = 0;
            dex.bSlaveHSDone = 0;
            gotoDexState( dexState_SendENQ, 1000 );
            break;
         }
#endif         
         printDexBytes( " rx", dex.rx_buf, dex.rx_size );
         if ( dex.rx_size > 0 && dex.rx_buf[ 0 ] == ENQ ) {           
           //u8 packet[] = {DLE, dex.bDleToggle? '1' : '0'};
           //uart3_tx( packet );

            dex.tx_size = 0;
            dex.tx_buf[ dex.tx_size++ ] = DLE;
            dex.tx_buf[ dex.tx_size++ ] = dex.bDleToggle? '1' : '0';

            dex.bDleToggle = !dex.bDleToggle;
            TX;

            dex.rx_size = 0;
            gotoDexState( dexState_SlaveHandshake2, 2000 );
            break;
         }
#if 0
         if ( dex.rx_size == 2 ) {
            if ( dex.rx_buf[ 0 ] == DLE && ( dex.rx_buf[ 1 ] == '1' || dex.rx_buf[ 1 ] == '0') ) {
               gotoDexState( dexState_MasterHandshake1, 100 );
               break;
            }
         }
#endif
         dex.rx_size = 0;
         gotoDexState( dexState_SlaveHandshake1, 1000 );
         break;

      case dexState_SlaveHandshake2:
         if ( !dex.rx_size ) {
            dex.bMasterHSDone = 0;
            dex.bSlaveHSDone = 0;
            gotoDexState( dexState_SendENQ, 1000 );
            break;
         }
         printDexBytes( " rx", dex.rx_buf, dex.rx_size );

         if ( dex.rx_buf[ 0 ] == DLE  &&  dex.rx_size > 3 && dex.rx_buf[ dex.rx_size-3 ] == ETX ){
            dex.tx_size = 0;
            dex.tx_buf[ dex.tx_size++ ] = DLE;
            dex.tx_buf[ dex.tx_size++ ] = dex.bDleToggle? '1' : '0';
            dex.bDleToggle = !dex.bDleToggle;
            TX;
            

            dex.rx_size = 0;
            gotoDexState( dexState_SlaveHandshake3, 1000 );
            break;
         }
         dex.rx_size = 0;
         gotoDexState( dexState_Idle, 1000 );
         break;

      case dexState_SlaveHandshake3:
         if ( !dex.rx_size ) {
            dex.bMasterHSDone = 0;
            dex.bSlaveHSDone = 0;
            gotoDexState( dexState_SendENQ, 1000 );
            break;
         }
         printDexBytes( " rx", dex.rx_buf, dex.rx_size );

         if ( dex.rx_buf[ 0 ] == EOT ){
               dex.bSlaveHSDone = 1;
               if ( dex.bMasterHSDone ) {
                  dex.audit_size = 0;
                  if ( dex.rx_size > 1 ) {
                     dex.numBlocksReceived = 0;
                     dex.blockDataSize = 0;

                     dex.tx_size = 0;
                     dex.tx_buf[ dex.tx_size++ ] = DLE;
                     dex.tx_buf[ dex.tx_size++ ] = dex.bDleToggle? '1' : '0';
                     dex.bDleToggle = !dex.bDleToggle;
                     TX;

                     gotoDexState( dexState_RecvAuditData1, 1000 );

                  } else {
                     gotoDexState( dexState_RecvAuditData1, 1000 );
                  }
               } else {
                  gotoDexState( dexState_MasterHandshake1, 1000 );
               }
               dex.rx_size = 0;
               break;
         }
         dex.rx_size = 0;
         gotoDexState( dexState_SlaveHandshake3, 1000 );
         break;

      case dexState_RecvAuditData1:
         if ( !dex.rx_size ) {
            log ( " rx_size = 0\r\n" );
            dex.bMasterHSDone = 0;
            dex.bSlaveHSDone = 0;
            gotoDexState( dexState_SendENQ, 1000 );
            log( " rx_size=0, to SendENQ\r\n" );
            break;
         }
         //printDexBytes( " rx: ", dex.rx_buf, dex.rx_size );

         if ( dex.rx_buf[ 0 ] == ENQ ) {

            dex.numBlocksReceived = 0;
            dex.blockDataSize = 0;

            dex.rx_size = 0;

            dex.tx_size = 0;
            dex.tx_buf[ dex.tx_size++ ] = DLE;
            dex.tx_buf[ dex.tx_size++ ] = dex.bDleToggle? '1' : '0';
            dex.bDleToggle = !dex.bDleToggle;
            TX;

            log( " ENQ received, to RecvAuditData1\r\n" );
            gotoDexState( dexState_RecvAuditData1, 1000 );
            break;
         }


         if ( ( dex.rx_size >= 6 ) && (dex.rx_buf[ 0 ] == DLE ) ) {
            if ( dex.rx_buf[ 1 ] == STX ) {
               if ( dex.rx_buf[ dex.rx_size - 4 ] == DLE ) {
                  //u16 chs = *((u16*)&dex.rx_buf[ dex.rx_size - 2 ]);
                  //printDexBytes( dex.rx_buf, dex.rx_size - 2 );
                  //log("<CHS:%x>\r\n", chs );

                  dex.blockDataSize = dex.rx_size - 6;

                  memcpy( &dex.audit_data[ dex.audit_size ], &dex.rx_buf[ 2 ], dex.blockDataSize );
                  dex.audit_size += dex.blockDataSize;

                  log( " block#%d of %d bytes, total:%d\r\n", dex.numBlocksReceived, dex.blockDataSize, dex.audit_size );

                  memset( dex.rx_buf, 0, sizeof(dex.rx_buf) );
                  dex.rx_size = 0;


                  dex.numBlocksReceived++;

                  //dex.bDleToggle = !dex.bDleToggle;

                  dex.tx_size = 0;
                  dex.tx_buf[ dex.tx_size++ ] = DLE;
                  dex.tx_buf[ dex.tx_size++ ] = dex.bDleToggle? '1' : '0';
                  dex.bDleToggle = !dex.bDleToggle;
                  TX;

                  gotoDexState( dexState_RecvAuditData1, 10 * 1000 );
                  break;
               } else {
                  log( " unexpected data\r\n" );
                  //log( " dex.rx_buf[ 1 ] = %x, dex.rx_buf[ dex.rx_size - 4 ]=%x\r\n", dex.rx_buf[ 1 ], dex.rx_buf[ dex.rx_size - 4 ] );
                  log( " [ 1 ] = %x, [ %d - 4 ] = %x\r\n", dex.rx_buf[ 1 ], dex.rx_size, dex.rx_buf[ dex.rx_size - 4 ] );
                  printDexBytes( "rx", dex.rx_buf, dex.rx_size );
               }
            }

         }

         if ( dex.rx_buf[ 0 ] == EOT ) {
            log( " %d bytes of audit data received in %d blocks\r\n", dex.audit_size, dex.numBlocksReceived );
            dex.rx_size = 0;
            dex.audit_data_valid = 1;

            dex.result = dexResult_Ok;
            //dex.dexResultTime = currTime;
            strcpy(dex.resultDescr, "ok" );

            dex.triggers.bits.query_cmplt = 1;

            gotoDexState( dex.state = dexState_Idle, 1000 );
            break;
         }

         dex.rx_size = 0;
         log(" unhandled data\r\n");
         dex.bSlaveHSDone = 0;
         dex.bMasterHSDone = 0;
         gotoDexState( dexState_SendENQ, 1000 );
         break;

      case dexState_Idle:
         break;

      default:
         log( " unexpected state\r\n" );
         dex.state = dexState_Initial;
         break;
	}
   log( "<read_dex(), st:%d-%s\r\n\r\n", dex.state, dexStateName[ dex.state ] );

   dex.sema = 0;
	return 0;
}

void calc_crc(u16 *pCrc, u8 uData) {
   u16 j;
   u16 BCC, BCC_0, BCC_1, BCC_14, DATA_0, X2, X15, X16;
   BCC = *pCrc;
   for ( j=0; j<8; j++) {
      DATA_0 = (uData >> j) & 0x0001;
      BCC_0 = (BCC & 0x0001);
      BCC_1 = (BCC >> 1) & 0x0001;
      BCC_14 = (BCC >> 14) & 0x0001;
      X16 = (BCC_0 ^ DATA_0) & 0x0001; 	// bit15 of BCC after shift
      X15  = (BCC_1 ^ X16) & 0x0001;		// bit0 of BCC after shift
      X2  = (BCC_14 ^ X16) & 0x0001;		// bit13 of BCC after shift
      BCC = BCC >> 1;
      BCC = BCC & 0x5FFE;
      BCC = BCC | (X15);
      BCC = BCC | (X2 << 13);
      BCC = BCC | (X16 << 15);
   }
   *pCrc = BCC;
}

void calc_crc2( char* uData, u16 size) {
   u16 i;
   for ( i = 0; i < size; i++) {
      if (uData[i] == DLE)
         continue;
      if (uData[i] == SOH)
         continue;
      if (uData[i] == STX)
         continue;

      calc_crc(&dex.crc, uData[i]);

      if (uData[i] == ETX)
         break;
   }
}
#if 0
u16 calc_crc3( char* uData, int size) {
   int i;
   u16 crc = 0;
   for ( i = 0; i < size; i++) {
      calc_crc( &crc, uData[i]);
   }
   return crc;
}
#endif





void printDexBytes( char* prefix, u8* buf, int size ) {
   int i;

   if ( prefix ) {
      log( "%s[%d]:", prefix, size );
   }

   for ( i = 0; i < size; i++ ) {

#if 1
      if ( size >= 6 ) {
         if ( ( i == size - 2 ) && ( buf[ i - 2 ] == DLE ) && (( buf[ i - 1 ] == ETB ) || ( buf[ i - 1 ] == ETX )) ){
            char chs_buf[ 16 ];
            u16 chs = 0;
            chs = buf[ i ] | buf[ i + 1 ]<<8;
            sprintf( chs_buf, "<CHS:%.4X>", chs );
            log( chs_buf );
            break;
         }
      }
#endif

      switch( buf[ i ]) {
         case NUL:
            log("<NUL>");
            break;
         case SOH:
            log("<SOH>");
            break;
         case STX:
            log("<STX>");
            break;
         case ETX:
            log("<ETX>");
            break;
         case EOT:
            log("<EOT>");
            break;
         case ENQ:
            log("<ENQ>");
            break;
         case ACK:
            log("<ACK>");
            break;
         case DLE:
            log("<DLE>");
            break;
         case NAK:
            log("<NAK>");
            break;
         case SYN:
            log("<SYN>");
            break;
         case ETB:
            log("<ETB>");
            break;
         case '\r':
            log("<CR>");
            break;
         case '\n':
            log("<LF>");
            break;
         default:
            //sprintf(mess, "%.2X ", buf[ i ]);
            //log( mess );
            log( "%c", buf[ i ] );
            break;
      }
   }
   log("\r\n");
}



void dexStartTimer( u32 timeout ) 
{
  OS_SetTimerPeriod( &dex.timer, timeout );
  OS_RetriggerTimer( &dex.timer );
}

void gotoDexState( int state, unsigned long timeout_ms )
{
   dex.state = state;
   dexStartTimer( timeout_ms );
   log(" to %s t:%d\r\n", dexStateName[ dex.state ], timeout_ms );
}

#if !defined ( DEX_SAMPLE )
extern char dex_sample;
#endif

void start_dex_query( void ) {
#if 0
   goto_cursor( 0x00 );
   lcd_print("starting dex   ");
#endif

   memset( dex.audit_data, 0, sizeof(dex.audit_data) );
   dex.audit_size = 0;
   dex.audit_data_valid = 0;
   dex.triggers.bits.query_cmplt = 0;

   dex.rx_size = 0;
   dex.tx_size = 0;

   dex.state = dexState_Initial;
   read_dex();
}

int abort_dex_query( void ) {
   int st = dex.state;
   dex.state = dexState_Idle;
   OS_StopTimer( &dex.timer );

   dex.rx_size = 0;
   dex.tx_size = 0;

#if 0
   goto_cursor( 0x00 );
   lcd_print("aborting dex   ");
#endif
   return st;
}

extern OS_TASK TCB3;

void dex_rx_isr(u8 b)
{
  if ( dex.rx_size < sizeof(dex.rx_buf) ) 
  {
     dex.rx_buf[ dex.rx_size++ ] = b;
  }
  if ( b == ENQ && dex.state != dexState_RecvAuditData1 )
  {
     OS_SignalEvent( EVT_DEX_DRD, &TCB3 );
  } 
  else if ( b == EOT && dex.rx_size == 1 )
  {
     OS_SignalEvent( EVT_DEX_DRD, &TCB3 );
  } 
  else  if ( dex.state != dexState_RecvAuditData1 && dex.rx_size >= 2 && dex.rx_buf[ dex.rx_size - 2 ] == DLE )
  {
     if ( b == '0' || b == '1' )
     {
        OS_SignalEvent( EVT_DEX_DRD, &TCB3 );           
     }
  } 
  else if ( dex.rx_size >= 4 && dex.rx_buf[ dex.rx_size - 4 ] == DLE ) 
  {
     if ( dex.rx_buf[ dex.rx_size - 3 ] == ETX || dex.rx_buf[ dex.rx_size - 3 ] == ETB )
     {
        OS_SignalEvent( EVT_DEX_DRD, &TCB3 );           
     }
  }  
}

u16 dex_tx_isr(u8* b)
{
  u16 rc = dex.tx_size;
  if( dex.tx_size > 0 ) 
  {
     *b = dex.tx_buf[ dex.tx_cnt++ ];
     dex.tx_size--;
  }
  else if( dex.tx_size == 0 ) 
  {
     dex.tx_cnt = 0;
  }  
  return rc;
}

void dex_isr( void* uart )
{
  if ( USART_GetFlagStatus( uart, USART_FLAG_RXNE ) ) 
  {
    // Push a new data into the receiver buffer
    volatile u8 b = USART_ReceiveData(uart);
    if ( dex.rx_size < sizeof(dex.rx_buf) ) 
    {
       dex.rx_buf[ dex.rx_size++ ] = b;
    }
    if ( b == ENQ && dex.state != dexState_RecvAuditData1 )
    {
       OS_SignalEvent( EVT_DEX_DRD, &TCB3 );
    } 
    else if ( b == EOT && dex.rx_size == 1 )
    {
       OS_SignalEvent( EVT_DEX_DRD, &TCB3 );
    } 
    else  if ( dex.state != dexState_RecvAuditData1 && dex.rx_size >= 2 && dex.rx_buf[ dex.rx_size - 2 ] == DLE )
    {
       if ( b == '0' || b == '1' )
       {
          OS_SignalEvent( EVT_DEX_DRD, &TCB3 );           
       }
    } 
    else if ( dex.rx_size >= 4 && dex.rx_buf[ dex.rx_size - 4 ] == DLE ) 
    {
       if ( dex.rx_buf[ dex.rx_size - 3 ] == ETX || dex.rx_buf[ dex.rx_size - 3 ] == ETB )
       {
          OS_SignalEvent( EVT_DEX_DRD, &TCB3 );           
       }
    }  
  }

  if ( USART_GetFlagStatus( uart, USART_FLAG_TXE ) && ( USART_GetITStatus( uart, USART_IT_TXE ) == SET ) ) 
  {
    if( dex.tx_size > 0 ) 
    {
      volatile u8 b =  dex.tx_buf[ dex.tx_cnt++ ];
      USART_SendData( uart, b );
      dex.tx_size--;
    }
    else if( dex.tx_size == 0 ) 
    {
      dex.tx_cnt = 0;
      USART_ITConfig( uart, USART_IT_TXE , DISABLE );         
    }  
  }  
}


#endif
