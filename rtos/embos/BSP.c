/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co KG                  *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2012  SEGGER Microcontroller GmbH & Co KG         *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system for microcontrollers      *
*                                                                    *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product or a real-time            *
*       operating system for in-house use.                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       OS version: 3.86d                                            *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : BSP.c
Purpose : BSP for STM32F103 (MB525)
--------  END-OF-HEADER  ---------------------------------------------
*/

#define BSP_C
#include "stm32f10x.h"
#include "BSP.h"
//#include "platform_config.h"
#include "sw_uart.h"
/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

/****** SFRs used for LED-Port **************************************/

#define _GPIOC_BASE_ADDR           (0x40011000)



GPIO_InitTypeDef GPIO_InitStructure;

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
#if defined LCD
#include "lcd_2x16.h"
#endif
#include "spi_slave.h"
#include "vending/pt-vending.h"

extern OS_RSEMA SemaLog;
extern OS_RSEMA SemaLCD;

OS_TICK_HOOK tickHook;


void hookFunc( void )
{
   IWDG_ReloadCounter();
}

////////////////////////////////////////////////////////////////////////////////


/*********************************************************************
*
*       BSP_Init()
*/
void BSP_Init(void) {
#if 0
  _SYSCTRL_RCC_APB2 |= (1uL << _SYSCTRL_LEDPORT_BIT);
  GPIOC_CRL  = (GPIOC_CRL & (0x00FFFFFFuL)) | 0x33000000uL;
  GPIOC_CRH  = (GPIOC_CRH & (0xFFFFFF00uL)) | 0x00000033uL;
  GPIOC_BRR |= _LED_MASK_ALL;   // Initially switch off all LEDS
#endif

#ifdef DEBUG
//#warning DEBUG
  //debug();
#else
//#warning NO DEBUG
#endif

   RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );
   RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE );   

   /* Enable USARTs clock */
   RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );
   RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );
   RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

   OS_CREATERSEMA( &SemaLog );    /* Creates resource semaphore  */
   OS_CREATERSEMA( &SemaLCD );    /* Creates resource semaphore  */
#if defined LCD   
   lcd_init();
#endif
  //spi_slave_init();
  //init_uarts();
  //init_fb();
   
#if 1 //defined ( SW_UART ) 
   /* Configure (PC.5) as alternate function push-pull */
   //HC05 CONF2
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_AF_PP;
   GPIO_Init(GPIOC, &GPIO_InitStructure);   
   HC05_CONF_OFF;
     
   sw_uart_init();
   
   
#endif
  
   OS_TICK_AddHook ( &tickHook, hookFunc );

}

#if 0
/*********************************************************************
*
*       LED switching routines
*       LEDs are switched on with low level on port lines
*/
void BSP_SetLED(int Index) {
  GPIOC_BSRR |= (1uL << (_LED0_BIT + Index));             /* Switch LED on */
}

void BSP_ClrLED(int Index) {
  GPIOC_BRR  |= (1uL << (_LED0_BIT + Index));             /* Switch LED off*/
}

void BSP_ToggleLED(int Index) {
  if ((GPIOC_ODR & (1uL << (_LED0_BIT + Index))) == 0) {  /* LED is switched off */
    BSP_SetLED(Index);
  } else {
    BSP_ClrLED(Index);
  }
}
#endif

/****** EOF *********************************************************/

