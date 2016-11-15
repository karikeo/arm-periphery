#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "pt-vending.h"
#include "dex.h"
#include "mdb.h"
#include "stdio.h"
#include "log.h"

#if defined LCD
#include "lcd_2x16.h"
#endif
#include "spi_comm.h"
#include "mdb_sniffer.h"
#include "..\sw_uart.h"
#include "..\bt.h"

const u8 verMajor = 1;
const u8 verMinor = 0;

t_fb_desc fb;
OS_Q  fb_evt_q;
char fb_evt_q_buf[ 64 ];

static GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;

//extern OS_RSEMA SemaLCD;
//OS_TIMER fbMasterTimer1;
//OS_TIMER fbSlaveTimer1;
OS_TIMER t_fbm_msg_end;
OS_TIMER t_fbs_msg_end;
//OS_TIMER fbSlaveTimer1;
OS_TIMER timer1000;

extern OS_TASK TCB2,TCB3;

#if defined DEX
extern tDex dex;
#elif defined MDB_SNIFFER
extern tMdbSniffer sniffer;
#endif

extern t_sw_uart sw_uart;

void timer1000_cb( void )
{
  if (LED2)
  {
    LED2_OFF;
  }
  else
  {
    LED2_ON;    
    //fb.cashless.display_tr = 1;    
  }
  OS_RetriggerTimer( &timer1000 );   /* Make timer periodical */  
  //mdb_test();  
}

#if 0
void fbMasterTimeout1( void )
{
   fb.m_tmo = 1;
   OS_SignalEvent( EVT_FB_M_RX_TMO, &TCB2 );
}

void fbSlaveTimeout1( void ) {
   fb.s_tmo = 1;
   OS_SignalEvent( EVT_FB_S_RX_TMO, &TCB2 );
}
#endif

void fbm_msg_end( void )
{
  fb.m.rx_msg_time = OS_GetTime_us();  
  
#if defined ( MDB_SNIFFER )  
  memmove( sniffer.m.raw.rx, fb.m.rx, fb.m.rx_size * sizeof(fb.m.rx[0]) );
  sniffer.m.raw.rx_size = fb.m.rx_size;
  sniffer.m.raw.time = fb.m.rx_last_time;
#endif
  OS_SignalEvent( EVT_FB_M_RX, &TCB2 );    
}

void fbs_msg_end( void )
{
#if defined ( MDB_SNIFFER )
  memmove( sniffer.s.raw.rx, fb.s.rx, fb.s.rx_size * sizeof(fb.s.rx[0]) );
  sniffer.s.raw.rx_size = fb.s.rx_size;
  sniffer.s.raw.time = fb.s.rx_last_time;  
#endif  
  OS_SignalEvent( EVT_FB_S_RX, &TCB2 );  
  memset(fb.s.rx, 0 , fb.s.rx_size);
  fb.s.rx_size = 0;
}

void Task_vending(void)
{
  OS_TASK_EVENT evt;
  //int i;

  /* LED2 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  LED2_OFF;
   
   init_fb();

   OS_CREATETIMER( &timer1000, timer1000_cb, 1000 );
   //OS_CREATETIMER( &fbMasterTimer1, fbMasterTimeout1, 5000 );
   //OS_CREATETIMER( &fbSlaveTimer1, fbSlaveTimeout1, 5000 );
   OS_CreateTimer( &t_fbm_msg_end, fbm_msg_end, 4 );
   OS_CreateTimer( &t_fbs_msg_end, fbs_msg_end, 5 );

   OS_ARM_ISRSetPrio( USART1_IRQn + 16, 140 );               // Set lowest Priority, ALL BITS set
   OS_ARM_ISRSetPrio( USART2_IRQn + 16, 140 );               // Set lowest Priority, ALL BITS set

   OS_ARM_EnableISR( USART1_IRQn + 16 );
   OS_ARM_EnableISR( USART2_IRQn + 16 );

   fb.type = eFbType_Unknown;
   fb.m_tmo = 0;
   fb.s_tmo = 0;
   fb.m.enabled = 0;

   memset( fb_evt_q_buf, 0, sizeof(fb_evt_q_buf) );
   OS_Q_Create(&fb_evt_q, &fb_evt_q_buf, sizeof(fb_evt_q_buf));

   enableLogging();
   log( "build: %u.%u %s %s\r\n", verMajor, verMinor,__DATE__, __TIME__);

      
#if 1
   sw_uart.rx.mes = '\r';
   sw_uart.rx.handler = bt_rx_msg_handler;
   
   OS_Delay (500);      

   sw_uart_tx("AT\r\n", 4);  
  //sw_uart_tx("+++", 3);     
   OS_Delay (500);
   
   HC05_CONF_ON;   
   OS_Delay (50);   
   
   sw_uart_tx("AT+NAME=VENDICONT\r\n", 19);
   OS_Delay (500);
         
   //sw_uart_tx("AT+ADDR?\r\n", 10);  
   //sw_uart_tx("AT+PSWD?\r\n", 10);  
   //OS_Delay (500);
   
   
   HC05_CONF_OFF;
   OS_Delay (500);
   
   sw_uart_tx("AT\r\n", 4);  
   OS_Delay (500);
   
   sw_uart_tx("AT\r\n", 4);  
   OS_Delay (500);
   
   
#endif
   
   sw_uart_reset_buffers();
   
   fb.s.rx_size = 0;
   memset(fb.s.rx,0,sizeof(fb.s.rx));
   fb.m.rx_size = 0;   
   memset(fb.m.rx,0,sizeof(fb.m.rx));
   
   OS_RetriggerTimer( &timer1000 );
   //OS_StartTimer( &fbMasterTimer1 );
   //OS_StartTimer( &fbSlaveTimer1 );
   
   fb.m.enabled = 1;
   fb.coin_acceptor.power_on = 1;

   while (1) {

      evt = OS_WaitEvent( //EVT_FB_EMU
                        //| EVT_DEX_DRD
                        //| 
                          EVT_FB_M_RX
                        | EVT_FB_S_RX
                        //| EVT_FB_M_RX_TMO
                        //| EVT_FB_S_RX_TMO
                        //| EVT_DEX_START
                        //| EVT_DEX_ABORT 
                        | EVT_CL_VEND_OK
                        | EVT_CL_VEND_CANCEL
                        | EVT_CL_VEND_FAIL
                        | EVT_CL_SESSION_COMPLETE
                          );


      if ( evt & EVT_FB_EMU ) {
         u32 evtType = 0;
         log( "fb_emu_evt=%d\r\n", fb.emu_evt );
         switch ( fb.emu_evt ) {
            case eFbEvtType_FbType:
               break;
            case eFbEvtType_DoorOpen:
               evtType = eFbEvtType_DoorOpen;
               OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
               log( "fb: door open\r\n" );
               break;
            case eFbEvtType_DoorClose:
               evtType = eFbEvtType_DoorClose;
               OS_Q_Put( &fb_evt_q, &evtType, sizeof( evtType ) );
               log( "fb: door close\r\n" );
               break;
         }
         fb.emu_evt = 0;
      }

      if ( evt & EVT_FB_M_RX_TMO ) {
         //log("fb_tx timeout\r\n");
      }

      if ( evt & EVT_FB_S_RX_TMO ) {
         //log("fb_rx timeout\r\n");
      }

      if ( evt & EVT_FB_M_RX )
      {
        //mdb_evt_m_rx();
#if defined ( MDB_SNIFFER )
        sniffer_m_rx();
#endif          
      }

      if ( evt & EVT_FB_S_RX )
      {
        //mdb_evt_s_rx();
#if defined ( MDB_SNIFFER )
        sniffer_s_rx();
#endif        
      }
      
      if ( evt & EVT_CL_VEND_OK )
      {
        u8  bt_buf[ 16 ];
        int bt_buf_sz = sprintf((char*)bt_buf, "VEND,OK,%d\n", fb.cashless.item_price);
        sw_uart_tx( bt_buf, bt_buf_sz );        
      }

      if ( evt & EVT_CL_VEND_CANCEL )
      {
        u8 bt_buf[] = "VEND,CANCEL\n";
        int bt_buf_sz = strlen((const char*)bt_buf);
        sw_uart_tx( bt_buf, bt_buf_sz );        
      }

      if ( evt & EVT_CL_VEND_FAIL )
      {
        u8 bt_buf[] = "VEND,FAIL\n";
        int bt_buf_sz = strlen((const char*)bt_buf);
        sw_uart_tx( bt_buf, bt_buf_sz );        
      }
      
      if ( evt & EVT_CL_SESSION_COMPLETE )
      {
        u8  bt_buf[ 16 ];
        int bt_buf_sz = sprintf((char*)bt_buf, "SESSION COMPLETE\n");
        sw_uart_tx( bt_buf, bt_buf_sz );                
      }

      if ( fb.en_log != fb.prev_en_log ) {
         //log( "fb mon is reset, %d\r\n", sizeof(mdb_mon) );
      }

      fb.prev_en_log = fb.en_log;
   }
}

////////////////////////////////////////////////////////////////////////////////
void init_fb( void ) 
{  
   /* Configure USART2 Tx (PA.2) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Configure USART1 Tx (PA.9) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOA, &GPIO_InitStructure);
  
   USART_Cmd( USART1, DISABLE );
   USART_Cmd( USART2, DISABLE );

   USART_DeInit( USART1 );
   USART_DeInit( USART2 );

   USART_InitStructure.USART_BaudRate = 9600;
   USART_InitStructure.USART_WordLength = USART_WordLength_9b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART1, &USART_InitStructure);
   USART_Init(USART2, &USART_InitStructure);   
   
   //USART_ITConfig( USART1, USART_IT_TXE, ENABLE );
   USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
   //USART_ITConfig( USART1, USART_IT_TC, ENABLE );   
   //USART_ITConfig( USART2, USART_IT_TC, ENABLE );
   USART_ITConfig( USART1, USART_IT_RXNE, ENABLE );
   USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
   
   USART_Init( USART1, &USART_InitStructure );
   USART_Init( USART2, &USART_InitStructure );

   USART_Cmd( USART1, ENABLE );
   USART_Cmd( USART2, ENABLE );
}
////////////////////////////////////////////////////////////////////////////////
void fb_s_tx( u16* p, u16 sz )
{
  memmove( &fb.s.tx[ fb.s.tx_size ], p, sz * sizeof(*p) );  
  fb.s.tx_size += sz;  
  USART_ITConfig( USART1, USART_IT_TXE , ENABLE );
}

void fb_m_tx( u16* p, u16 sz )
{
  memcpy( &fb.m.tx[ fb.m.tx_size ], p, sz * sizeof(*p));  
  fb.m.tx_size += sz;
  USART_ITConfig( USART2, USART_IT_TXE , ENABLE );
}

/* USART1 Interrupt ----------------------------------------------*/
__irq void USART1_IRQHandler(void) {

  OS_EnterNestableInterrupt();

  if ( USART_GetFlagStatus( USART1, USART_FLAG_ORE ) )
  {
    USART_ClearFlag( USART1, USART_FLAG_ORE );
  }
  if ( USART_GetFlagStatus( USART1, USART_FLAG_FE ) )
  {
    USART_ClearFlag( USART1, USART_FLAG_FE );
  }
  if ( USART_GetFlagStatus( USART1, USART_FLAG_NE ) )
  {
    USART_ClearFlag( USART1, USART_FLAG_NE );
  }
  if ( USART_GetFlagStatus( USART1, USART_FLAG_PE ) )
  {
    USART_ClearFlag( USART1, USART_FLAG_PE );
  }
  
  if ( USART_GetFlagStatus( USART1, USART_FLAG_RXNE ) )
  {
    u32 curr_time = OS_GetTime_us();    
    volatile u16 b = USART_ReceiveData( USART1 );
    if ( fb.s.rx_size < MDB_MAX_LEN )
    {
      fb.s.rx[ fb.s.rx_size++ ] = b;
      fb.s.rx_last_time = curr_time;
      if ( fb.s.rx_size == 0 )
      {
        fb.s.rx_start_time = curr_time;        
      }
    }

    fb.s_tmo = 0;
    
    if ( b & MDB_MODE_MASK )
    {
      fb.s.rx_msg_time = curr_time;
      //OS_StopTimer( &t_fbs_msg_end );
      //fbs_msg_end();
    }
    //else
    //{
      OS_RetriggerTimer( &t_fbs_msg_end );      
    //}   
#if defined ( MDB_PROXY ) && ( MDB_PROXY == BILL_VALIDATOR )    
    //b |= 0x100;
    fb_m_tx((u16*)&b, 1);
    //fb_m_tx(&fb.s.rx[ fb.s.rx_size-1 ], 1);    
#endif
    
  }

  //Transmit
  if ( USART_GetFlagStatus( USART1, USART_FLAG_TXE ) && ( USART_GetITStatus( USART1, USART_IT_TXE ) == SET ) )
  {
    if( fb.s.tx_size > 0 ) 
    {
      volatile u16 b =  fb.s.tx[ fb.s.tx_cnt++ ];
      USART_SendData( USART1, b );
      fb.s.tx_size--;
    }
    else if( fb.s.tx_size == 0 ) 
    {
      fb.s.tx_cnt = 0;
      USART_ITConfig( USART1, USART_IT_TXE , DISABLE );         
    }  
  }

  OS_LeaveNestableInterrupt();
}
////////////////////////////////////////////////////////////////////////////////
/* USART2 Interrupt ----------------------------------------------*/
//Tx Loop
__irq void USART2_IRQHandler(void)
{
  //static u8 tr = 0;

  OS_EnterNestableInterrupt();
  //OS_EnterInterrupt();
   
  if ( USART_GetFlagStatus( USART2, USART_FLAG_ORE ) )
  {
    USART_ClearFlag(USART2,USART_FLAG_ORE);
  }
  if ( USART_GetFlagStatus( USART2, USART_FLAG_FE ) )
  {
    USART_ClearFlag(USART2,USART_FLAG_FE);
  }
  if ( USART_GetFlagStatus( USART2, USART_FLAG_NE ) )
  {
    USART_ClearFlag(USART2,USART_FLAG_NE);
  }
  if ( USART_GetFlagStatus( USART2, USART_FLAG_PE ) )
  {
    USART_ClearFlag(USART2,USART_FLAG_PE);
  }
  if ( USART_GetFlagStatus( USART2, USART_FLAG_LBD ) )
  {
    USART_ClearFlag(USART2,USART_FLAG_LBD);
  }
      
  if ( USART_GetFlagStatus( USART2, USART_FLAG_RXNE ) )
  {
    u8 skip = 0;
    //USART_ClearFlag(USART2,USART_FLAG_RXNE);
    volatile u16 b = USART_ReceiveData(USART2);
    u32 curr_time = OS_GetTime_us();
    if ( ( b & MDB_MODE_MASK ) || fb.m.rx_size >= MDB_MAX_LEN )
    {
      fb.m.rx_size = 0;
      memset( fb.m.rx, 0, sizeof(fb.m.rx) );
      
      fb.m.rx_start_time = curr_time;
      fb.m.rx_msg_time = 0;
      fb.m.rx_last_time = 0;
      memset(fb.m.rx_times,0,sizeof(fb.m.rx_times));
    }
    
    if ( fb.m.rx_last_time && curr_time - fb.m.rx_last_time > 2000 )
    {
      skip = 1;
    }
    else
    {
      OS_RetriggerTimer( &t_fbm_msg_end );      
    }
    
    if ( fb.m.rx_size < MDB_MAX_LEN && !skip )
    {
      fb.m.rx_times[ fb.m.rx_size ] = curr_time;
      fb.m.rx[ fb.m.rx_size++ ] = b;
      mdb_evt_m_rx();
    }
    
    fb.m.rx_last_time = curr_time;

  }
   
  //Transmit
  if ( USART_GetFlagStatus( USART2, USART_FLAG_TXE ) && ( USART_GetITStatus( USART2, USART_IT_TXE ) == SET ) )
  {
    if( fb.m.tx_size > 0 ) 
    {
      volatile u16 b =  fb.m.tx[ fb.m.tx_cnt++ ];
      USART_SendData( USART2, b );
      fb.m.tx_size--;
    }
    else if( fb.m.tx_size == 0 )
    {
      fb.m.tx_cnt = 0;      
      USART_ITConfig( USART2, USART_IT_TXE , DISABLE );
    }  
  }

  OS_LeaveNestableInterrupt();
  //OS_LeaveInterrupt();  
}
////////////////////////////////////////////////////////////////////////////////
/* USART3 Interrupt ----------------------------------------------*/
__irq void USART3_IRQHandler(void) {

   OS_EnterNestableInterrupt();

   if ( USART_GetFlagStatus( USART3, USART_FLAG_ORE ) ) 
   {// Overrun Error
      USART_ClearFlag(USART3,USART_FLAG_ORE);      
   }
   if ( USART_GetFlagStatus( USART3, USART_FLAG_FE ) ) 
   {// Framing Error
      USART_ClearFlag(USART3,USART_FLAG_FE);      
   }
   if ( USART_GetFlagStatus( USART3, USART_FLAG_NE ) ) 
   {// Noise Error
      USART_ClearFlag(USART3,USART_FLAG_NE);      
   }
   if ( USART_GetFlagStatus( USART3, USART_FLAG_PE ) ) 
   {// Parity Error
      USART_ClearFlag(USART3,USART_FLAG_PE);      
   }
   // Push a new data into the receiver buffer
#if defined DEX
  dex_isr(DEX);
#elif defined MDB_SNIFFER
  sniffer_isr(MDB_SNIFFER);
#else
   if ( USART_GetFlagStatus( USART3, USART_FLAG_RXNE ) ) 
   {
      // Push a new data into the receiver buffer
      volatile u8 b = USART_ReceiveData(USART3);
   }
#endif     
   
   OS_LeaveNestableInterrupt();
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void USART3_Timeout( void )
{
#if defined DEX
   OS_SignalEvent( EVT_DEX_DRD, &TCB3 );
#elif defined MDB_SNIFFER
   OS_SignalEvent( EVT_SNIFF_RX_MSG, &TCB3 );
#endif  
}

void Task_USART3(void) 
{
#if defined ( MDB_SNIFFER ) || defined ( DEX )
  OS_TASK_EVENT evt;
#endif
  
   /* Configure USART3 Tx (PB.10) as alternate function push-pull */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
   
   USART_InitStructure.USART_BaudRate = 230400;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   USART_Init(USART3, &USART_InitStructure);

   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
#if defined (MDB_SNIFFER) || defined (DEX)
   USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
#endif   
   USART_Cmd(USART3, ENABLE);

   OS_ARM_ISRSetPrio( USART3_IRQn + 16, 140 );               // Set lowest Priority, ALL BITS set
   OS_ARM_EnableISR( USART3_IRQn + 16 );

#if defined ( DEX )
   OS_CreateTimer( &dex.timer, USART3_Timeout, 1000 );  
   dex_init();
   //dex.state = dexState_Initial;
   //read_dex();  
#elif defined ( MDB_SNIFFER )
   OS_CreateTimer( &sniffer.timer, USART3_Timeout, 2 );    
#endif
  
   OS_Delay (500);

   while (1) 
   {
#if defined ( DEX )
      evt = OS_WaitEvent( EVT_DEX_DRD | EVT_DEX_START | EVT_DEX_ABORT );    
      if ( evt & EVT_DEX_DRD ) 
      {
         read_dex();
      }

      if ( evt & EVT_DEX_START ) 
      {
         start_dex_query();
      }

      if ( evt & EVT_DEX_ABORT ) 
      {
         abort_dex_query();
      }
#elif defined ( MDB_SNIFFER )
      evt = OS_WaitEvent( EVT_SNIFF_RX_MSG );
      if ( evt & EVT_SNIFF_RX_MSG ) 
      {
         sniffer_rx_msg();
      }    
#else
      OS_Delay (500);    
#endif
   }
}
