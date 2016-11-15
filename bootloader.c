/******************** (C) COPYRIGHT 2013 **************************************
* File Name          : bootloader.c
* Author             : Antipov Ilya
* Version            : V1.0.0
* Date               : 04/03/2013
* Description        : Bootloader
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include "stdio.h"
#include "string.h"
#include "stm32f10x_lib.h"
#include "intrinsics.h"
#include "spi_comm.h"
#include "lcd_2x16.h"



/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
//typedef __task void (*pEntryPoint)(void);
/* Private define ------------------------------------------------------------*/
#define BufferSize 32

/* Private macro -------------------------------------------------------------*/
#define u8 unsigned char

/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
SPI_InitTypeDef SPI_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

u16 gpiob_idr = 0;
u8 spi_resync = 0;
u32 spi_cs_wait = 0;

t_spi_out spi_out[2];
t_spi_out prev_spi_out;
t_spi_in spi_in[2];
__absolute t_spi_mgr spi_mgr_desc;

u32 pages_erased[4] = {0,0,0,0};

/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void spi_slave_init();
void configure_spi(void);
__task void jump(void);

u32 fw_total_size = 0;
u32 fw_programmed = 0;

u32 led1_cnt = 0;

u8 trigger1 = 0;
u8 trigger2 = 0; //timeout waiting for not Idle State
u8 trigger3 = 0; //

u8 buf[ 64 ];
////////////////////////////////////////////////////////////////////////////////
int spi_mgr( ) {
   u32 adr;
   u32 i;
   //do {
      switch ( spi_mgr_desc.state ) {

         case spiState_Idle:
            switch ( spi_out[1].fields.cmd ) {
               case spiCmd_FwUpdateInit:
                  spi_in[0].fields.status.bits.spi_busy = 1;
                  spi_mgr_desc.state =  spiState_FwWrite;
                  spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;

                  spi_mgr_desc.state =  spiState_FwWrite;
                  fw_total_size = spi_out[1].fields.param1;
                  fw_programmed = 0;
                  spi_in[0].fields.status.bits.spi_busy = 0;
                  break;

               default:
                  spi_mgr_desc.state = spiState_Idle;
                  break;
            }
            break;

         case spiState_FwWrite:
            switch ( spi_out[1].fields.cmd ) {
               case spiCmd_FwUpdateData: {
                  u32 err_adr = 0;
                  u32 err_code = 0;

                  spi_in[0].fields.status.bits.spi_busy = 1;
                  spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
                  adr = spi_out[1].fields.param1; //0x08000000 +

                  if ( adr >= 0x08004000 ) {

                     FLASH_Status rc = 0;

                     if ( adr == 0x8004000 + 0x400 ) {
                        rc = -2;
                     }


                     if ( adr == 0x8007A60 ) {
                        rc = -2;
                     }


                     u32 pageNo = ( adr - 0x08000000 ) / 0x400;
                     u8 pb = pageNo / 32;
                     if ( ( pages_erased[ pb ] & ( 1 << ( pageNo - pb * 32 ) ) ) == 0 ) {
                        FLASH_Unlock();
                        FLASH_ErasePage( 0x08000000 + pageNo * 0x400 );
                        pages_erased[ pb ] |=  1 << ( pageNo - pb * 32 );
                     }

                    for ( i = 0; i < spi_out[1].fields.param2/4; i++ ) {
                        u32 u = ((u32*)spi_out[1].fields.data)[i];
                        rc = FLASH_ProgramWord( adr + i*4, u );

                        if ( rc != FLASH_COMPLETE) {
                           err_adr = adr + i * 4;
                           err_code = rc;
                           sprintf( buf, "a:%x e:%d", err_adr, rc );
                           goto_cursor( 0x40 );
                           lcd_print( buf );
                        }

                    }
                     fw_programmed +=  spi_out[1].fields.param2;
                 }

                 spi_in[0].fields.param1 = spi_out[1].fields.param1;
                 spi_in[0].fields.param2 = spi_out[1].fields.param2;
                 spi_in[0].fields.status.bits.spi_busy = 0;

                 trigger1  = 1;

                 break;
               }

               case spiCmd_FwUpdateComplete:
                  spi_in[0].fields.status.bits.spi_busy = 1;
                  spi_mgr_desc.state = spiState_FwWriteComplete;
                  spi_in[0].fields.state = spi_out[1].fields.cmd | spiCmd_Ack;
                  spi_in[0].fields.param1 = 0;
                  spi_in[0].fields.param2 = 0;
                  spi_in[0].fields.status.bits.spi_busy = 0;
                  spi_mgr_desc.state = spiState_Idle;
                  trigger3 = 0;
                  break;

               default:
                  spi_mgr_desc.state = spiState_Idle;

            }
            break;

         case spiState_FwWriteComplete:
            spi_mgr_desc.state = spiState_Idle;
            break;
      }
   //} while ( bLoop );

   return 0;
}
////////////////////////////////////////////////////////////////////////////////
/* DMA Channel4 Interrupt ----------------------------------------------*/
__irq void DMA1_Channel4_IRQHandler(void) {
   u8 valid = 0;

   if ( spi_out[1].fields.padding[ 1 ] == 'i'
     && spi_out[1].fields.padding[ 2 ] == 'v'
     && spi_out[1].fields.padding[ 3 ] == 'a'
   ) {
      valid = 1;
   }

   if ( valid ) {

      if ( spi_out[1].fields.cmd != prev_spi_out.fields.cmd
          || spi_out[1].fields.param1 !=  prev_spi_out.fields.param1 ) {
         memcpy( prev_spi_out.raw, spi_out[1].raw, sizeof( spi_out[1].raw ) );
         spi_mgr();
      }

   }

   if ( ( GPIOB->IDR & GPIO_Pin_12 ) != GPIO_Pin_12 && !valid ) {
      //SPI_Cmd(SPI2, DISABLE);
      spi_resync = 1;
      gpiob_idr = GPIOB->IDR;
   }

   DMA1->IFCR |= DMA1_IT_GL4;
}
////////////////////////////////////////////////////////////////////////////////
/* DMA Channel4 Interrupt ----------------------------------------------*/
__irq void DMA1_Channel5_IRQHandler(void) {

   DMA1->IFCR |= DMA1_IT_GL5;

   spi_in[0].fields.status.bits.bootloader = 1;

   spi_in[0].fields.padding[ 0 ] = spi_out[1].fields.padding[ 0 ];
   spi_in[0].fields.padding[ 1 ] = 'i';
   spi_in[0].fields.padding[ 2 ] = 'v';
   spi_in[0].fields.padding[ 3 ] = 'a';

   spi_in[0].fields.status.bits.error = trigger2;

   trigger3 = 1;

}
////////////////////////////////////////////////////////////////////////////////
void init(void) {
  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

  spi_slave_init();

}
////////////////////////////////////////////////////////////////////////////////
void deinit(void) {

   /* RCC system reset(for debug purpose) */
   RCC_DeInit();

   /* NVIC configuration ------------------------------------------------------*/
   NVIC_DeInit();
   NVIC_SCBDeInit();

   /* GPIO configuration ------------------------------------------------------*/
   GPIO_DeInit(GPIOB);

   SPI_I2S_DeInit(SPI2);

   DMA_DeInit(DMA1_Channel4);
   DMA_DeInit(DMA1_Channel5);
}

////////////////////////////////////////////////////////////////////////////////
__task void load(void) {
#ifdef DEBUG
   debug();
   lcd_init();
   //lcd_write_control( 0x01 );
#endif

   deinit();
   init();

   memset( pages_erased, 0, sizeof(pages_erased));

   __enable_interrupt ();

#if 1
   trigger2 = 0;
   for ( volatile u32 i = 0; spi_mgr_desc.state == spiState_Idle ; i++ ) {

      IWDG_ReloadCounter();

      if ( spi_resync ) {
         for ( spi_cs_wait = 0; spi_cs_wait < 20; spi_cs_wait++ ) {
            if ( GPIOB->IDR & GPIO_Pin_12 )
               break;
            DelayuS( 1000 );
         }

         configure_spi();
         spi_resync = 0;
      }

      if ( i > 0x100000 ) {
         trigger2 = 1;
      }

      if ( led1_cnt > 1000 * 200 ) {
         GPIO_SetBits(GPIOC, GPIO_Pin_7);
         led1_cnt = 0;
      } else led1_cnt++;


      if ( led1_cnt > 1000 * 100 ) {
         GPIO_ResetBits(GPIOC, GPIO_Pin_7);
      }
   }
#endif

   for ( volatile u32 i = 0; spi_mgr_desc.state != spiState_Idle ; i++ ) {

      IWDG_ReloadCounter();

      if ( spi_resync ) {
         for ( spi_cs_wait = 0; spi_cs_wait < 20; spi_cs_wait++ ) {
            if ( GPIOB->IDR & GPIO_Pin_12 )
               break;
            DelayuS( 1000 );
         }

         configure_spi();
         spi_resync = 0;
      }

      if ( i >= 1000 * 100 ) {
         i = 0;

         sprintf( buf, "loading...%u%%", ( fw_programmed * 100 ) / fw_total_size );
         goto_cursor(0x00);
         lcd_print( buf );

         if ( trigger1 ) {
            trigger1 = 0;
         } else {

         }
      }

      if ( led1_cnt > 1000 * 100 ) {
         GPIO_SetBits(GPIOC, GPIO_Pin_7);
         led1_cnt = 0;
      } else led1_cnt++;


      if ( led1_cnt > 1000 * 50 ) {
         GPIO_ResetBits(GPIOC, GPIO_Pin_7);
      }
   }


   sprintf( buf, "loading...%u%%", ( fw_programmed * 100 ) / fw_total_size );
   goto_cursor(0x00);
   lcd_print( buf );

   //for ( volatile u32 i = 0; i < 1024 * 1024 ; i++ );
   //for ( volatile u32 i = 0; spi_mgr_desc.state == spiState_Idle ; i++ );
   for ( volatile u32 i = 0; trigger3 == 0 ; i++ );
#if 0
   deinit();
   jump();
#else
   NVIC_GenerateSystemReset();
#endif
}
////////////////////////////////////////////////////////////////////////////////
__task void jump(void) {
   pEntryPoint ep;
  __disable_interrupt();
  __set_MSP(*((u32*)0x08004000));
  ep = (pEntryPoint) (*((u32*)0x08004004));
  ep();
}
////////////////////////////////////////////////////////////////////////////////
__task int main(void) {
#ifdef DEBUG
   debug();
   lcd_init();
#endif

  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)  {
    /* IWDGRST flag set */
    /* Turn on LED1 */
    /* Clear reset flags */
      RCC_ClearFlag();
      lcd_print( "wdg" );
      load();
  }  else {
    /* IWDGRST flag is not set */
    /* Turn off LED1 */
  }

   for ( ; ; ) {
     jump();
   }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/






////////////////////////////////////////////////////////////////////////////////
void spi_slave_init() {

  memset( spi_out, 0 , sizeof( spi_out ) );
  memset( spi_in, 0 , sizeof( spi_in ) );
  memset( &prev_spi_out, 0 , sizeof( prev_spi_out ) );
  memset( &spi_mgr_desc, 0 , sizeof( spi_mgr_desc ) );

  configure_spi();

  spi_mgr_desc.state = spiState_Idle;
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
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)spi_in;
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

void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK/2 */
    RCC_PCLK2Config(RCC_HCLK_Div2);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08)
    {}
  }

#if 1
  /* Enable peripheral clocks --------------------------------------------------*/
  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

  /* SPI2 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

#endif
}

void GPIO_Configuration(void) {
   GPIO_InitTypeDef GPIO_InitStructure;

   /* Configure SPI2 pins: SCK(PB13) and MOSI(PB15) */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   //LED1
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
  /* Set the Vector Table base location at 0x20000000 */
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Configure and enable SPI1 interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable SPI2 interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable DMA1 channel 4 interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable DMA1 channel 5 interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

}

