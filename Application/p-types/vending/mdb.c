#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "pt-vending.h"
#include "cashless.h"
#include "stdio.h"
#include "stdlib.h"
#include "log.h"
#if defined LCD
  #include "lcd_2x16.h"
#endif
#include "spi_comm.h"

#include "mdb.h"
#include "mdb_sniffer.h"
#include "..\sw_uart.h"

extern t_fb_desc fb;
t_mdb_mon mdb_mon;

extern OS_Q  fb_evt_q;
       
extern tMdbSniffer sniffer;

static u8 calc_crc(u16*, u8);

extern OS_TASK TCB2;

void mdb_evt_s_rx()
{
  //int i;
  //int len = 0;
  //u8 bLog = 0;
  //t_mdb_session *d = 0;
  
#if 0
  sprintf( buf,"[%.2x]", fbSlaveRxBuf[ 0 ]);
  goto_cursor( 0x4C );
  lcd_print( buf );
#endif
   
#if defined ( MDB_SNIFFER )
  //sniffer_s_rx();
#endif  
   
#if 0  
   switch ( fb.m.rx[ 0 ] & 0xFF )
   {
   case MDB_CMD_CHG_TUBE_STATUS:  //0x0A
      d = &mdb_mon.sessions[ 0 ];
      break;
   case MDB_CMD_CHG_COIN_TYPE:    //0x0C
      d = &mdb_mon.sessions[ 1 ];
      if ( fb.m.rx_size == 6 ) {
         u32 evtType = 0;

         if ( fb.m.rx[ 3 ] == 0xFF && fb.m.rx[ 4 ] == 0xFF
            && d->rqst[ 3 ] == 0x00 && d->rqst[ 4 ] == 0x00 ) {
            evtType = eFbEvtType_DoorOpen;
            OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
#if 0
            log( "#1:%d %.2X %.2X %.2X %.2X %.2X %.2X "
                , d->rqst_size
                , d->rqst[ 0 ], d->rqst[ 1 ], d->rqst[ 2 ], d->rqst[ 3 ], d->rqst[ 4 ], d->rqst[ 5 ] );
#endif
            log( "fb: door open\r\n" );

         } else if ( fb.m.rx[ 3 ] == 0x00 && fb.m.rx[ 4 ] == 0x00
            && d->rqst[ 3 ] == 0xFF && d->rqst[ 4 ] == 0xFF ) {
            evtType = eFbEvtType_DoorClose;
            OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
            log( "fb: door closed\r\n" );
         }

      }
#if 0
      log( "#2:%d %.2X %.2X %.2X %.2X %.2X %.2X\r\n"
          , d->rqst_size
          , d->rqst[ 0 ], d->rqst[ 1 ], d->rqst[ 2 ], d->rqst[ 3 ], d->rqst[ 4 ], d->rqst[ 5 ] );

#endif
#if 0
      log( "#3:%d %.2X %.2X %.2X %.2X %.2X %.2X\r\n"
          , fb.m.rx_size
          , fb.m.rx[ 0 ], fb.m.rx[ 1 ], fb.m.rx[ 2 ], fb.m.rx[ 3 ], fb.m.rx[ 4 ], fb.m.rx[ 5 ] );
      bLog = 1;
#endif
      break;
    case MDB_CMD_CHG_POLL:         //0x0B
      d = &mdb_mon.sessions[ 2 ];
      break;
    case MDB_CMD_BV_POLL:          //0x33
      d = &mdb_mon.sessions[ 3 ];
      break;
    case MDB_CMD_BV_STACKER:       //0x36
      d = &mdb_mon.sessions[ 4 ];
      break;
    case MDB_CMD_BV_BILLTYPE:      //0x34
      d = &mdb_mon.sessions[ 5 ];
      break;
    default:
      bLog = 1;
      break;
  }
#if 1
   if ( fb.en_log && !bLog && d ) {
      u8 ms = 0;
      u8 mc = 0;
      ms = d->rqst_size != fb.m.rx_size;
      mc = memcmp( d->rqst, fb.m.rx, fb.m.rx_size * 2 );
#if 0
      if ( fb.m.rx[ 0 ] == 0x10C ) {
         log("ms=%d,%d,%d\r\n", ms, d->rqst_size, fb.m.rx_size);
         log("mc=%d\r\n", mc);
         bLog = 1;
      }
#endif
      if ( ms || mc ) {
#if 1
         if( ms && fb.en_log )
            log("ms(%d<>%d) ", d->rqst_size, fb.m.rx_size);
         if ( mc && fb.en_log )
            log("mc " );
#endif
         bLog = 1;

      }

      ms = d->rply_size != fb.s.rx_size;
      mc = memcmp( d->rply, fb.s.rx, fb.s.rx_size * 2 );
      if ( ms || mc ) {
#if 0
         if( ms && fb.en_log )
            log("ss ");
         if ( mc && fb.en_log )
            log("sc ");
#endif
         bLog = 1;
      }
   }
#endif
   if ( fb.en_log && bLog ) {

      log( "fbm: " );
      for ( i = 0; i < fb.m.rx_size; i++ ) {
         log( "%.2x ", fb.m.rx[ i ] );
      }
      log( "\r\n" );

      if ( d ) {
         log( "_bm: " );
         for ( i = 0; i < d->rqst_size; i++ ) {
            log( "%.2x ", d->rqst[ i ] );
         }
         log( "\r\n" );
      }

      log( "fbs: " );
      for ( i = 0; i < fb.s.rx_size; i++ ) {
         log( "%.2x ", fb.s.rx[ i ] );
      }
      log( "\r\n" );
   }

   if ( d ) {
      len = ( fb.m.rx_size < MDB_MAX_LEN ) ? fb.m.rx_size : MDB_MAX_LEN;
      memcpy( d->rqst, fb.m.rx, len * 2 );
      d->rqst_size = len;

      len = ( fb.s.rx_size < MDB_MAX_LEN ) ? fb.s.rx_size : MDB_MAX_LEN;
      memcpy( d->rply, fb.s.rx, len * 2 );
      d->rply_size = len;
   }
#endif   
   fb.s.rx_size = 0;
}

void mdb_evt_m_rx()
{      
  static u8 just_reset = 1;
  static u8 tr = 1;
  
  if ( !fb.m.enabled || fb.m.rx_size == 0 )
  {
    return;
  }
  
  u8 n = 0;
  u16 b;
  u16 buf[ MDB_MAX_LEN ];  
  
  u8 cashless1 = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x110 );
  u8 cashless2 = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x160 );
  u8 coin_changer = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x108 );
  //u8 coin_hopper1 = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x158 );  
  //u8 coin_hopper2 = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x170 );
  //u8 bill_validator = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x130 ); 
  u8 usd = ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x140 ) 
    || ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x148 )
    || ( ( fb.m.rx[ 0 ] & 0x1F8 ) == 0x150 );  
  
  if ( usd || cashless1 || cashless2 )
  {
    tr = 1;
  }
   
  if ( coin_changer )
  {    
    tr = 1;
  }
#if 0  
  switch( fb.m.rx[ 0 ] & 0x1F8 )
  {
    case ( MDB_ADR_CASHLESS1 | 0x100 ):
      n = cashless_mdb_evt_rx(buf);
      break;
  }
#endif
  
#if defined ( MDB_PROXY ) && ( MDB_PROXY == BILL_VALIDATOR )  
  if ( bill_validator )
  {
      b = fb.m.rx[ fb.m.rx_size - 1 ];
      buf[ n++ ] = b;      
      fb_s_tx( buf, n );
      return;
  }
#endif
  
#if defined ( MDB_CASHLESS1 )  
  
#endif
  
  switch(  fb.m.rx[ 0 ] )
  {        
#if defined ( MDB_COIN_CHANGER )
    case 0x108:
      if ( fb.m.rx_size >= 2 )
      {
        buf[ n++ ] = 0x100;
        fb.coin_acceptor.just_reset = 1;
      }
      break;
    
    case 0x10B:
      if ( fb.m.rx_size >= 2 )
      {
        if ( fb.coin_acceptor.power_on )
        {
          fb.coin_acceptor.power_on = 0;
          buf[ n++ ] = 0x0B;
          buf[ n++ ] = 0x10B;          
        }        
        else if ( fb.coin_acceptor.just_reset )
        {
          fb.coin_acceptor.just_reset = 0;
          buf[ n++ ] = 0x00;      
          buf[ n++ ] = 0x100;
        }
        else
        {
          buf[ n++ ] = 0x100;
        }
      }
      break;
      
    case 0x109: 
      if ( fb.m.rx_size >= 2 )
      {
          buf[ n++ ] = 0x02; //level
          buf[ n++ ] = 0x11; //country code
          buf[ n++ ] = 0x52;
          buf[ n++ ] = 0x05; //scaling
          buf[ n++ ] = 0x00; //decimal places
          buf[ n++ ] = 0x00; //coin type routing
          buf[ n++ ] = 0x3E;
          buf[ n++ ] = 0x01; //coin type credit
          buf[ n++ ] = 0x02;          
          buf[ n++ ] = 0x0A;          
          buf[ n++ ] = 0x14;          
          buf[ n++ ] = 0x14;
          buf[ n++ ] = 0x64;          
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;
      }
      break;      
      
    case 0x10F: 
      if ( fb.m.rx_size >= 3 )
      {
        if (  fb.m.rx[ 1 ] == 0x00 )
        {        
          buf[ n++ ] = 0x4E;
          buf[ n++ ] = 0x52;
          buf[ n++ ] = 0x49;
          buf[ n++ ] = 0x31;
          buf[ n++ ] = 0x30;
          buf[ n++ ] = 0x31;
          buf[ n++ ] = 0x30;
          buf[ n++ ] = 0x34;
          buf[ n++ ] = 0x37;
          buf[ n++ ] = 0x32;
          buf[ n++ ] = 0x34;
          buf[ n++ ] = 0x2D;
          buf[ n++ ] = 0x31;
          buf[ n++ ] = 0x35;
          buf[ n++ ] = 0x36;
          buf[ n++ ] = 0x43;
          buf[ n++ ] = 0x32;
          buf[ n++ ] = 0x20;
          buf[ n++ ] = 0x76;
          buf[ n++ ] = 0x36;
          buf[ n++ ] = 0x20;
          buf[ n++ ] = 0x45;
          buf[ n++ ] = 0x20;
          buf[ n++ ] = 0x20;
          buf[ n++ ] = 0x33;
          buf[ n++ ] = 0x37;
          buf[ n++ ] = 0x37;
          buf[ n++ ] = 0x09;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x0F;
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;
        }
        else if (  fb.m.rx[ 1 ] == 0x01 )
        {//feature enable
          buf[ n++ ] = 0x100;
        }
      }
      break;      

    case 0x10A: 
      if ( fb.m.rx_size >= 2 )
      {
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x04;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x07;
        buf[ n++ ] = 0x11;
        buf[ n++ ] = 0x04;
        buf[ n++ ] = 0x1C;
        buf[ n++ ] = 0x10;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        buf[ n++ ] = 0x00;
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;        
      }
      break;
      
    case 0x10C: 
      if ( fb.m.rx_size >= 6 )
      {
        buf[ n++ ] = 0x100;        
      }
      break;
#endif
      
#if defined ( MDB_CASHLESS1 )
    case 0x110:
    //case 0x160:     
      if ( fb.m.rx_size >= 2 )
      {
        buf[ n++ ] = 0x100;
        just_reset = 1;
      }
      break;
      
    case 0x112:
    //case 0x162:      
      if ( fb.m.rx_size >= 2 )
      {
        if ( just_reset )
        {
          just_reset = 0;
          buf[ n++ ] = 0x00;      
          buf[ n++ ] = 0x100;        
        }
        else if ( fb.cashless.display_tr )
        {
          fb.cashless.display_tr = 0;
          buf[ n++ ] = 0x02;
          buf[ n++ ] = 0x80;
          buf[ n++ ] = 'H';
          buf[ n++ ] = 'o';
          buf[ n++ ] = 'l';
          buf[ n++ ] = 'a';
          buf[ n++ ] = ' ';          
          buf[ n++ ] = 'D';
          buf[ n++ ] = 'a';
          buf[ n++ ] = 'n';
          buf[ n++ ] = 'i';
          buf[ n++ ] = 'e';
          buf[ n++ ] = 'l';
          buf[ n++ ] = '!';
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!'; 
          buf[ n++ ] = 'H';
          buf[ n++ ] = 'o';
          buf[ n++ ] = 'l';
          buf[ n++ ] = 'a';
          buf[ n++ ] = ' ';          
          buf[ n++ ] = 'A';
          buf[ n++ ] = 'l';
          buf[ n++ ] = 'v';
          buf[ n++ ] = 'a';
          buf[ n++ ] = 'r';
          buf[ n++ ] = 'o';
          buf[ n++ ] = '!';
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!';          
          buf[ n++ ] = '!'; 
/*          
          for ( int i = n ; i < 32 + 2; i++ )
          {
            buf[ n++ ] = '+'; 
          }
*/          
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;
        }
        else if ( fb.cashless.begin_session_tr )
        {
          fb.cashless.begin_session_tr = 0;
          buf[ n++ ] = 0x03;
          buf[ n++ ] = ( fb.cashless.funds_available >> 8) & 0xFF;
          buf[ n++ ] = fb.cashless.funds_available  & 0xFF;
          buf[ n++ ] = 0x01;
          buf[ n++ ] = 0x02;
          buf[ n++ ] = 0x03;
          buf[ n++ ] = 0x04;                      
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x00;
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;                               
        }
        else if ( fb.cashless.cancel_session_tr )
        {
          fb.cashless.cancel_session_tr = 0;
          buf[ n++ ] = 0x04;
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;           
        }        
        else
        {
          buf[ n++ ] = 0x100;
        }        
      }
      break;   
      
    case 0x111:
    //case 0x161:      
      //01 02 00 56 01 00 0F 03 16C
      if ( fb.m.rx_size >= 7 )
      {
        if (  fb.m.rx[ 1 ] == 0x00 )
        {
          buf[ n++ ] = 0x01;
          buf[ n++ ] = 0x02;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x56;
          buf[ n++ ] = 0x01;
          buf[ n++ ] = 0x00;
          buf[ n++ ] = 0x0F;
          buf[ n++ ] = 0x03;
          b = calc_crc(buf, n);
          buf[ n++ ] = 0x100 | b;
        }
        else if (  fb.m.rx[ 1 ] == 0x01 )
        {
          buf[ n++ ] = 0x100;
        }
        else
        {
          buf[ n++ ] = 0x1FF;        
        }
      }
      else
      {
        
      }
      break;      
      
    case 0x113:
    //case 0x163:
      //113 00 00 01 00 01 15
      //113 02 00 64 79
      //113 04 17 
      //tr = 0;      
      if ( fb.m.rx_size >= 7 && fb.m.rx[1] == 0x00 )
      {
        fb.cashless.item_price = ( fb.m.rx[2] << 8 ) | fb.m.rx[3];
        fb.cashless.item_no = ( fb.m.rx[4] << 8 ) | fb.m.rx[5];
        buf[ n++ ] = 0x05;
        buf[ n++ ] = ( fb.cashless.item_price >> 8 ) & 0xFF;        
        buf[ n++ ] = fb.cashless.item_price & 0xFF;
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;                  
      }            
      else if ( fb.m.rx_size >= 3 && fb.m.rx[1] == 0x01 )
      { //VEND CANCEL
        buf[ n++ ] = 0x06;        
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;           
        OS_SignalEvent( EVT_CL_VEND_CANCEL, &TCB2 );                
      }                  
      else if ( fb.m.rx_size >= 5 && fb.m.rx[1] == 0x02 )
      {
        b = 0;
        buf[ n++ ] = 0x100 | b;          
        OS_SignalEvent( EVT_CL_VEND_OK, &TCB2 );
      }
      else if ( fb.m.rx_size >= 5 && fb.m.rx[1] == 0x03 )
      {
        b = 0;
        buf[ n++ ] = 0x100 | b;          
        OS_SignalEvent( EVT_CL_VEND_FAIL, &TCB2 );
      }      
      else if ( fb.m.rx_size >= 3 && fb.m.rx[1] == 0x04 )
      {
        buf[ n++ ] = 0x07;        
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;                 
        OS_SignalEvent( EVT_CL_SESSION_COMPLETE, &TCB2 );        
      }      
      break;
      
    case 0x115:      
    //case 0x165:
      if ( fb.m.rx_size >= 3 && fb.m.rx[1] == 0x00 )
      {
        buf[ n++ ] = 0x0E;
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;        
      }      
      else if ( fb.m.rx_size >= 3 && fb.m.rx[1] == 0x01 )
      {
        buf[ n++ ] = 0x0F;
        buf[ n++ ] = 0x24;//0x15;
        buf[ n++ ] = 0x94;//0x26;
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;        
      }            
      break;
        
    case 0x114:
    //case 0x164:      
      if ( fb.m.rx_size >= 3 )
      {
        fb.cashless.enabled = ( fb.m.rx[1] != 0 );
        buf[ n++ ] = 0x100;
      }
      break;   
      
    case 0x117:      
    //case 0x167:
      if ( fb.m.rx_size >= 32 )
      {
        buf[ n++ ] = 0x09;
        for ( int i  = 1; i < 30; i++)
        {
          buf[ n++ ] = 0;
        }
        b = calc_crc(buf, n);
        buf[ n++ ] = 0x100 | b;
      }      
      break;
#endif      
  }
  
  if ( n > 0 )
  {
    fb_m_tx( buf, n );    
  }
}

static u8 calc_crc(u16* p, u8 n)
{
  u8 crc = 0;
  for ( u8 i = 0; i < n; i++ )
  {
    crc += p[ i ];
  }
  return crc;
}

#if 0

u16 voteForEXE = 0;
u16 voteForMDB = 0;

void detectFbType( u16 b ) {
   u8 calcedParity = 0;
   u8 receivedParity = ( ( b & 0x0100 ) != 0 );
   int i;
   for ( i = 0; i < 8; i++ ) {
      if ( b & ( 1 << i ) )
         //calcedParity = !calcedParity;
         calcedParity = ( calcedParity != 0);
   }

   if ( ( fb.m.rx_size == 1 && receivedParity )
       || ( fb.m.rx_size > 1 ) && !receivedParity && calcedParity ) {
      if ( voteForMDB < 20 ) {
         voteForMDB++;
      } else if ( fb.type != eFbType_MDB ) {
         fb.type = eFbType_MDB;
         //u32 evtType = eFbEvtType_FbType;
         //OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
      }
      if ( voteForEXE > 0 ) {
         voteForEXE--;
      }

   } else if ( receivedParity == calcedParity ) {
      if ( voteForMDB > 0 ) {
         voteForMDB--;
      }
      if ( voteForEXE < 20 ) {
         voteForEXE++;
      } else if ( fb.type != eFbType_EXE ) {
         fb.type = eFbType_EXE;
         //u32 evtType = eFbEvtType_FbType;
         //OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
      }
   }

#if 0
   if( fb.en_log )
      log("b=%X ", b );

   if ( ( receivedParity && calcedParity )
       || ( !receivedParity && !calcedParity ) ) {
      if ( voteForEXE < 20 ) {
         voteForEXE++;
         if( fb.en_log )
            log("e+ r%d c%d\r\n" , receivedParity, calcedParity );
      } else {
         fb.type = eFbType_EXE;
      }
   } else {
      if ( voteForEXE > 0 ) {
         voteForEXE--;

         if( fb.en_log )
            log("e- r%d c%d\r\n" , receivedParity, calcedParity );
      } else {
         fb.type = eFbType_MDB;
      }
   }
#endif
}
#endif
