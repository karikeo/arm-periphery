#include "stm32f10x.h"
#include "stm32f10x_dbgmcu.h"
#include "RTOS.h"
#include "BSP.h"
#include "spi_slave.h"
#include "spi_comm.h"
#if defined LCD
#include "lcd_2x16.h"
#endif
#include "stdio.h"
#include "log.h"
#include "vending/pt-vending.h"
#include "vending/dex.h"
#include "vending/mdb.h"
#include "utils.h"

extern const u8 verMajor;
extern const u8 verMinor;

extern GPIO_InitTypeDef GPIO_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
//EXTI_InitTypeDef EXTI_InitStructure;
//NVIC_InitTypeDef NVIC_InitStructure;

extern OS_TASK TCB1; //SPI
extern OS_TASK TCB2; //Vending
extern OS_Q fb_evt_q;
extern t_fb_desc fb;
#if defined DEX
extern tDex dex;
#endif   


#ifdef __ICCARM__  // IAR
   __task void jump(void);
   __absolute t_spi_in spi_in[2];
   __absolute t_spi_out spi_out[2];   
#elif __CC_ARM    // KEIL
   void jump(void);
   t_spi_in spi_in[2];
   t_spi_out spi_out[2];   
#elif __GNUC__    // GCC
  #warning watch me
#endif

t_spi_out prev_spi_out;


//OS_MAILBOX spi_out_mb;
//extern OS_RSEMA SemaLCD;
OS_TIMER timer100;
OS_TIMER spi_ok_timer;
OS_TIMER spi_sync_timer;

u16 gpiob_idr;
u32 spi_cs_wait = 0;

//#define SPI_OUT_MB_SIZE 4
//u8 spi_out_mb_buf[ sizeof(t_spi_out) * SPI_OUT_MB_SIZE ];

#define ISR_ID_SYSTICK                (15)               // System Tick

////////////////////////////////////////////////////////////////////////////////
#ifdef __ICCARM__  // IAR
   __absolute t_spi_mgr spi_mgr_desc;
#elif __CC_ARM    // KEIL
   t_spi_mgr spi_mgr_desc;
#elif __GNUC__    // GCC
  #warning watch me
#endif
////////////////////////////////////////////////////////////////////////////////
void timer100_cb( void ) {

   if ( spi_out[1].fields.padding[ 1 ] == 'i'
     && spi_out[1].fields.padding[ 2 ] == 'v'
     && spi_out[1].fields.padding[ 3 ] == 'a'
   ) {
      //GPIO_ResetBits(GPIOC, GPIO_Pin_9);
   } else {
      //GPIO_SetBits(GPIOC, GPIO_Pin_9);
   }

   IWDG_ReloadCounter();
   OS_RetriggerTimer( &timer100 );   /* Make timer periodical */
   
   OS_SignalEvent( SPI_TASK_EVT_100MS, &TCB1 );

}
////////////////////////////////////////////////////////////////////////////////
void spi_ok_timer_cb( void ) {
   //char buf[ 32 ];
   //GPIO_SetBits(GPIOC, GPIO_Pin_9);
   OS_RetriggerTimer( &timer100 );
   //log("spi timeout\r\n");
}
////////////////////////////////////////////////////////////////////////////////
void spi_sync_timer_cb( void ) {
//   if ( DMA1_Channel5->CNDTR < 31 ) {
//   }
}
////////////////////////////////////////////////////////////////////////////////
#ifdef __ICCARM__  // IAR
__task void jump(void) {
   pEntryPoint ep;
   disable_interrupt();
   NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x4000 );
   __set_MSP(*((u32*)0x08000000));
   ep = (pEntryPoint) (*((u32*)(0x08000000 + 7*4) ));
   ep();
}
#elif __CC_ARM    // KEIL
void jump(void) {
   pEntryPoint ep;
   disable_interrupt();
   NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x4000 );
   __set_MSP(*((u32*)0x08000000));
   ep = (pEntryPoint) (*((u32*)(0x08000000 + 7*4) ));
   ep();
}
#elif __GNUC__    // GCC
  #warning watch me
#endif
////////////////////////////////////////////////////////////////////////////////
int spi_mgr( ) {
//   u32 adr;
//   u32 i;
   char buf[ 32 ];

   switch ( spi_mgr_desc.state ) {

      case spiState_Idle:
         switch ( spi_out[1].fields.cmd ) {
            case spiCmd_FwUpdateInit:
#if defined LCD
               goto_cursor( 0x00 );
               lcd_print("to bootloader  ");
#endif
               OS_ARM_DisableISR( ISR_ID_SYSTICK );
               disable_interrupt();
               jump();
               for ( ; ; ) {
                  ;
               }
               break;

            case spiCmd_FwUpdateData:
               break;

            case spiCmd_FwUpdateComplete:
               break;

            case spiCmd_GetFwVersion:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               spi_in[0].fields.param1 = verMajor;
               spi_in[0].fields.param2 = verMinor;
               build_dt( buf );
               buf[ sizeof(spi_in[0].fields.data) - 1 ] = 0;
               memcpy( spi_in[0].fields.data, buf, sizeof(spi_in[0].fields.data) );
               spi_in[0].fields.status.bits.spi_busy = 0;               
               break;


            case spiCmd_ReadLog:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               spi_in[0].fields.param1 =
                  logRead( (char*)spi_in[0].fields.data, sizeof( spi_in[0].fields.data ));
               spi_in[0].fields.status.bits.spi_busy = 0;
               break;

            case spiCmd_NoCmd:
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               spi_in[0].fields.param1 = 0;
               spi_in[0].fields.param2 = 0;
               memset( spi_in[0].fields.data, 0, sizeof( spi_in[0].fields.data ));
               spi_in[0].fields.status.bits.spi_busy = 0;
               spi_mgr_desc.state = spiState_Idle;
               break;

#if defined DEX                 
            case spiCmd_DexQueryStart:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               OS_SignalEvent( EVT_DEX_START, &TCB2 );
               spi_in[0].fields.param1 =  dex.state;
               spi_in[0].fields.status.bits.spi_busy = 0;
               break;

            case spiCmd_DexQueryAbort:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               OS_SignalEvent( EVT_DEX_ABORT, &TCB2 );
               spi_in[0].fields.param1 =  dex.state;
               spi_in[0].fields.status.bits.spi_busy = 0;
               break;

            case spiCmd_DexQueryRead: {
               u32 sizeLeft = 0;
               u32 chunkSize = sizeof(spi_in[0].fields.data);
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));

               if ( spi_out[1].fields.param1 == 0 ) {
                  dex.audit_ptr = 0;
               }
               sizeLeft = dex.audit_size - dex.audit_ptr;

               if ( sizeLeft <  chunkSize )
                  chunkSize = sizeLeft;

               memcpy( spi_in[0].fields.data, &dex.audit_data[ dex.audit_ptr ], chunkSize );
               spi_in[0].fields.param1 =  dex.audit_ptr;
               spi_in[0].fields.param2 =  sizeLeft;

               dex.audit_ptr += chunkSize;

               spi_in[0].fields.status.bits.spi_busy = 0;
#if 0 //defined LCD
               goto_cursor(0x00);
               sprintf( (char*)buf, "tx %u of %u", (char*)dex.audit_ptr, (char*)dex.audit_size );
               lcd_print( buf );
#endif
               if ( sizeLeft == 0 ) {
                  dex.triggers.bits.query_cmplt = 0;
               }
               break;
            }
#endif
            
            case spiCmd_GetFbEvent: {
               int len;
               u8* p = 0;               
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               len = OS_Q_GetPtrCond( &fb_evt_q, (void**)&p );           /* Check message   */
               if ( len > 0 ) {
                  if ( len > 16 )
                     len = 16;
                  spi_in[0].fields.param1 =  ((u32*) p ) [ 0 ];

                  OS_Q_Purge( &fb_evt_q );
               } else {

               }
#if 0
               log( "l=%d %d %d\r\n"
                   ,len
                   ,spi_in[0].fields.status.bits.fb_evt
                   ,OS_Q_GetMessageCnt( &fb_evt_q )
               );
#endif
               spi_in[0].fields.status.bits.spi_busy = 0;
               break;
            }
            case spiCmd_EmuFbEvent:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               fb.emu_evt = spi_out[1].fields.param1;
               OS_SignalEvent( EVT_FB_EMU, &TCB2 );
               spi_in[0].fields.status.bits.spi_busy = 0;
               log( "fb_emu: %d\r\n", fb.emu_evt );
               break;
               
            case spiCmd_ReadData:
               spi_in[0].fields.status.bits.spi_busy = 1;
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               memset( spi_in[0].fields.data, 0, sizeof(spi_in[0].fields.data));
               switch( spi_out[1].fields.param1 ) {
                  case 0:
                  case 1:                    
                     spi_in[0].fields.param1 = spi_out[1].fields.param1;
                     spi_in[0].fields.param2 = spi_out[1].fields.param2;
                     extern float t_18b20;                     
                     *((float*)(spi_in[0].fields.data)) = t_18b20;
                     break;
                  default:
                     spi_in[0].fields.param1 = 0;
                     spi_in[0].fields.param2 = 0;
                     break;
               }
               spi_in[0].fields.status.bits.spi_busy = 0;
               break;
               
            default:
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;              
               spi_in[0].fields.status.bits.spi_busy = 0;              
               break;
         }
         break;
         
      default:
         spi_in[0].fields.status.bits.spi_busy = 0;              
         break;
        
   }

   return 0;
}

////////////////////////////////////////////////////////////////////////////////
void Task_spi(void) {
   OS_TASK_EVENT evt;
#if defined LCD
   char buf[ 64 ];   
   goto_cursor(0x00);
   sprintf( (char*)buf, "hallo %d.%d", verMajor, verMinor );
   lcd_print((u8*)buf);
   goto_cursor(0x40);
   sprintf( buf, "%s%s", __DATE__, __TIME__);
   lcd_print((u8*)buf);

   //OS_Delay (500);

   lcd_clear();
#endif
   spi_slave_init();

   OS_ARM_ISRSetPrio( DMA1_Channel4_IRQn + 16, 140 );   // Set lowest Priority, ALL BITS set
   OS_ARM_ISRSetPrio( DMA1_Channel5_IRQn + 16, 140 );   // Set lowest Priority, ALL BITS set
   OS_ARM_EnableISR( SPI2_IRQn + 16 );
   OS_ARM_EnableISR( DMA1_Channel4_IRQn + 16 );
   OS_ARM_EnableISR( DMA1_Channel5_IRQn + 16 );

   OS_CreateTimer( &timer100, timer100_cb, 100 );
   OS_CreateTimer( &spi_ok_timer, spi_ok_timer_cb, 2000 );
   OS_CreateTimer( &spi_sync_timer, spi_sync_timer_cb, 50 );
   OS_StartTimer( &timer100 );
   OS_StartTimer( &spi_ok_timer );

   for ( ; ; ) {

      evt = OS_WaitEvent( SPI_TASK_EVT_RX
                        | SPI_TASK_EVT_RX_SYNC
                        | SPI_TASK_EVT
                        | SPI_TASK_EVT_100MS );

      if ( evt & SPI_TASK_EVT_RX_SYNC ) {

         for ( spi_cs_wait = 0; spi_cs_wait < 20; spi_cs_wait++ ) {
            if ( GPIOB->IDR & GPIO_Pin_12 )
               break;
            OS_Delay(1);
         }

         configure_spi();
#if defined LCD
         sprintf( buf, "[%.4X] %lu", gpiob_idr, spi_cs_wait );
         goto_cursor( 0x40 );
         lcd_print( buf );
#endif
         OS_Delay(1);
         log("spi resync, %lums\r\n", spi_cs_wait );
      }

      if ( evt & SPI_TASK_EVT ) {
         spi_mgr( );
      }

      if ( evt & SPI_TASK_EVT_RX ) {
#if defined LCD        
#if 0
         sprintf( buf,"[%.2X]", spi_out[1].fields.padding[0]);
         goto_cursor( 0x40 );
         lcd_print( buf );
#endif
#if 0
         sprintf( buf, "[%lu]", DMA1_Channel5->CNDTR );
         goto_cursor( 0x04 );
         lcd_print( buf );
#endif
#endif         
      }

      if ( evt & SPI_TASK_EVT_100MS ) {
#if defined LCD
         sprintf( buf, "[%.2X]%.2X%.2X%.2X"
                 , spi_out[1].raw[ 28 ]
                 , spi_out[1].raw[ 29 ]
                 , spi_out[1].raw[ 30 ]
                 , spi_out[1].raw[ 31 ]
         );
         goto_cursor( 0x06 );
         lcd_print( buf );
#endif
      }
   }
}
////////////////////////////////////////////////////////////////////////////////
/* DMA Channel4 Interrupt ----------------------------------------------*/
__irq void DMA1_Channel4_IRQHandler(void) {

   u8 valid;
   u8 changes;
   
   OS_EnterNestableInterrupt();

   valid = 0;
   changes = 0;
   
   if ( spi_out[1].fields.padding[ 1 ] == 'i'
     && spi_out[1].fields.padding[ 2 ] == 'v'
     && spi_out[1].fields.padding[ 3 ] == 'a'
   ) {
      LED4_ON;
      OS_RetriggerTimer( &spi_ok_timer );
      valid = 1;
   } else {
      LED4_OFF;
   }

   changes = ( spi_out[1].fields.cmd !=  prev_spi_out.fields.cmd )
          || ( spi_out[1].fields.param1 !=  prev_spi_out.fields.param1 );

   if ( valid && changes ) {
      memcpy( prev_spi_out.raw, spi_out[1].raw, sizeof( spi_out[1].raw ) );
      if ( spi_mgr_desc.state == spiState_Idle ){
         u16 cmd = spi_out[1].fields.cmd;
         switch ( cmd ) {
            case spiCmd_NoCmd:
            case spiCmd_FwUpdateInit:
            case spiCmd_ReadLog:
            case spiCmd_DexQueryStart:
            case spiCmd_DexQueryAbort:
            case spiCmd_DexQueryRead:
            case spiCmd_GetFwVersion:
            case spiCmd_EmuFbEvent:
            case spiCmd_GetFbEvent:
            case spiCmd_ReadData:
               OS_SignalEvent( SPI_TASK_EVT, &TCB1 );
               //spi_mgr();
               break;

            default:
               spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
               break;
         }
      }
   }

   fb.en_log = spi_out[1].fields.control.bits.en_fb_log;
#if defined DEX   
   dex.enable = spi_out[1].fields.control.bits.en_dex;
#endif
   DMA1->IFCR |= DMA1_IT_GL4;

   if ( valid ) {
      OS_SignalEvent( SPI_TASK_EVT_RX, &TCB1 );
   } else if ( ( GPIOB->IDR & GPIO_Pin_12 ) == 0 ) {
      OS_SignalEvent( SPI_TASK_EVT_RX_SYNC, &TCB1 );
      gpiob_idr = GPIOB->IDR;     
   }
   OS_LeaveNestableInterrupt();
}
////////////////////////////////////////////////////////////////////////////////
/* DMA Channel5 Interrupt ----------------------------------------------*/
__irq void DMA1_Channel5_IRQHandler(void) {

   OS_EnterNestableInterrupt();

   spi_in[0].fields.status.bits.bootloader = 0;
   spi_in[0].fields.status.bits.error = 0;
   spi_in[0].fields.status.bits.nda = 0;
   spi_in[0].fields.status.bits.log = ( isLogEmpty() >  0 );
#if defined DEX     
   spi_in[0].fields.status.bits.dex_rdy = ( dex.state == dexState_Idle );
   spi_in[0].fields.status.bits.dex_cmplt = dex.triggers.bits.query_cmplt;
#endif   
   //spi_in[0].fields.status.bits.ve_inhibit = ?
   spi_in[0].fields.status.bits.fb_m_tmo = fb.m_tmo;
   spi_in[0].fields.status.bits.fb_s_tmo = fb.s_tmo;
   spi_in[0].fields.status.bits.fb_type = fb.type;
   spi_in[0].fields.status.bits.fb_evt = OS_Q_GetMessageCnt( &fb_evt_q ) > 0;

   spi_in[0].fields.padding[ 0 ] = spi_out[1].fields.padding[ 0 ];
   spi_in[0].fields.padding[ 1 ] = 'i';
   spi_in[0].fields.padding[ 2 ] = 'v';
   spi_in[0].fields.padding[ 3 ] = 'a';

   DMA1->IFCR |= DMA1_IT_GL5;

   OS_RetriggerTimer( &spi_sync_timer );
   OS_LeaveNestableInterrupt();
}
////////////////////////////////////////////////////////////////////////////////
/* SPI2 Interrupt ----------------------------------------------*/
__irq void SPI2_IRQHandler(void) {
}
////////////////////////////////////////////////////////////////////////////////
void configure_spi(void) {

   DMA_DeInit(DMA1_Channel4);
   DMA_DeInit(DMA1_Channel5);
   SPI_I2S_DeInit(SPI2);

/* DMA Channel4 Configuration ----------------------------------------------*/
   DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DR;
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)spi_out[0].raw;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_BufferSize = 2 * SPI_RX_SIZE;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel4, &DMA_InitStructure);

/* DMA Channel5 Configuration ----------------------------------------------*/
   memset(&DMA_InitStructure, 0, sizeof(DMA_InitStructure));
   DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI2->DR;
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)spi_in[0].raw;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_BufferSize = SPI_TX_SIZE;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;//
   DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel5, &DMA_InitStructure);

   DMA_Cmd(DMA1_Channel4, ENABLE );
   DMA_Cmd(DMA1_Channel5, ENABLE );

/* Config SPI[2] = Master */
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_High;//SPI_CPOL_Low
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//SPI_CPHA_2Edge
   SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;//;SPI_NSS_Soft
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial = 1;
   SPI_Init(SPI2, &SPI_InitStructure);

   SPI_I2S_ITConfig( SPI2, SPI_I2S_IT_RXNE | SPI_I2S_IT_TXE | SPI_I2S_IT_ERR, ENABLE );

   SPI2->CR2 = SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

   DMA1_Channel4->CCR |= 3;
   DMA1_Channel5->CCR |= 3;

   /* Enable SPI2 */
   SPI_Cmd(SPI2, ENABLE);
}
////////////////////////////////////////////////////////////////////////////////
void spi_slave_init() {

   memset( spi_out[0].raw, 0 , sizeof( spi_out[0].raw ) );
   memset( spi_out[1].raw, 0 , sizeof( spi_out[1].raw ) );
   memset( spi_in[0].raw, 0 , sizeof( spi_in[0].raw ) );
   memset( spi_in[1].raw, 0 , sizeof( spi_in[1].raw ) );
   memset( prev_spi_out.raw, 0 , sizeof( prev_spi_out.raw ) );
   memset( &spi_mgr_desc, 0 , sizeof( spi_mgr_desc ) );

   /* Enable write access to IWDG_PR and IWDG_RLR registers */
   IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
   /* IWDG counter clock: 40KHz(LSI) / 32 = 1.25 KHz */
   IWDG_SetPrescaler( IWDG_Prescaler_32 );
   /* Set counter reload value to 349 */
   IWDG_SetReload(349*3);
   /* Reload IWDG counter */
   IWDG_ReloadCounter();
   /* Enable IWDG (the LSI oscillator will be enabled by hardware) */

   DBGMCU_Config( DBGMCU_IWDG_STOP, ENABLE );
   IWDG_Enable();

#if 1
/* Configure SPI2 pins: SCK(PB13) and MOSI(PB15) */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //LED3
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   GPIO_ResetBits(GPIOC, GPIO_Pin_9);   
   GPIO_SetBits(GPIOC, GPIO_Pin_9);
#endif

   configure_spi();

}


void HardFault_Handler   (void) {
   //const u16 KR_KEY_Reload = (u16)0xAAAA;
   const u16 KR_KEY_Enable = (u16)0xCCCC;

   IWDG->KR = IWDG_WriteAccess_Enable;
   IWDG->PR = IWDG_Prescaler_4;
   IWDG->RLR = 1;
   IWDG->KR = KR_KEY_Enable;
   //NVIC_GenerateSystemReset();
}


#if 0
/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__irq void EXTI15_10_IRQHandler(void) {
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
   //SelFunc();
    /* Clear the EXTI Line 15 */
   EXTI_ClearITPendingBit(EXTI_Line15);
  }
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    //DownFunc();
    /* Clear the EXTI Line 13 */
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
      /* SEL function */
      //UpFunc();
      /* Clear the EXTI Line 14 */
      EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
#endif
