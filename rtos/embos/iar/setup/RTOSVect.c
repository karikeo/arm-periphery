/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co KG                  *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2014  SEGGER Microcontroller GmbH & Co KG         *
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
*       OS version: 4.02a                                            *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : RTOSVect.c
Purpose : Vector table for STM32F103 (IAR STM32-SK), CMSIS compatible. 
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "exceptions.h"    // CMSIS compatible exception handler definition
#include "RTOS.h"

#ifdef __cplusplus
  extern "C" {
#endif

/*********************************************************************
*
*       Reference to other modules
*
**********************************************************************
*/

#ifdef __ICCARM__
  #pragma language=extended
  #if (__VER__ < 500)
    #pragma segment="CSTACK"
    extern void __program_start(void);
  #else
    #pragma section="CSTACK"
    extern void __iar_program_start(void);
  #endif  // #if (__VER__ < 500)
#endif    // #ifdef __ICCARM__

#ifdef __CC_ARM
  extern unsigned int Image$$CSTACK$$ZI$$Limit;
  extern void __main(void);
#endif

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define NVIC_HFSR          *((volatile unsigned long *)(0xE000ED2CuL))

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

/*********************************************************************
*
*       _IllegalException()
*
*       Is called from any exception handler which is not implemented
*       by the application (not overwriiten by user).
*       We implemented an endless loop here, so the programm stops
*       when any of the exceptions is called.
*       Using a debugger and setting a breakpoint here allows to
*       analyze which exception / interrupt was called.
*       Normally, the CPU should not arrive here.
*/
static void _IllegalException(void) {
  while(1) {  // Unhandled exception
  }
}

/*********************************************************************
*
*       Dummy system exception handler
*
*       We implemented a call to _IllegalException(), so the program
*       calls this function when any of the exceptions is called.
*       Using a debugger and setting a breakpoint in _IllegelException()
*       allows to analyze which exception / interrupt was called.
*/
WEAK void NMI_Handler         (void) { _IllegalException(); }
WEAK void MemManage_Handler   (void) { _IllegalException(); }
WEAK void BusFault_Handler    (void) { _IllegalException(); }
WEAK void UsageFault_Handler  (void) { _IllegalException(); }
WEAK void SVC_Handler         (void) { _IllegalException(); }
WEAK void DebugMon_Handler    (void) { _IllegalException(); }
WEAK void PendSV_Handler      (void) { _IllegalException(); }
WEAK void SysTick_Handler     (void) { _IllegalException(); }

WEAK void HardFault_Handler   (void) {
  //
  // In case we received a hard fault because
  // of a breakpoint instruction, we return.
  // This may happen when using semihosting for printf.
  //
  if (NVIC_HFSR & (1uL << 31)) {
    NVIC_HFSR |=  (1uL << 31);   // Reset hardfault status
    OS_HardFaultHandler();
    return;                      // Return to interrupted application
  }
  //
  // In other cases call the dummy handler for unimplemented interrupts
  //
 _IllegalException();
}

/*********************************************************************
*
*       Default dummy interrupt handler
*
*       We implemented a call to _IllegalException(), so the program
*       calls this function when any of the exceptions is called.
*       Using a debugger and setting a breakpoint in _IllegelException()
*       allows to analyze which exception / interrupt was called.
*/
WEAK void WWDG_IRQHandler           (void) { _IllegalException(); }
WEAK void PVD_IRQHandler            (void) { _IllegalException(); }
WEAK void TAMPER_IRQHandler         (void) { _IllegalException(); }
WEAK void RTC_IRQHandler            (void) { _IllegalException(); }
WEAK void FLASH_IRQHandler          (void) { _IllegalException(); }
WEAK void RCC_IRQHandler            (void) { _IllegalException(); }
WEAK void EXTI0_IRQHandler          (void) { _IllegalException(); }
WEAK void EXTI1_IRQHandler          (void) { _IllegalException(); }
WEAK void EXTI2_IRQHandler          (void) { _IllegalException(); }
WEAK void EXTI3_IRQHandler          (void) { _IllegalException(); }
WEAK void EXTI4_IRQHandler          (void) { _IllegalException(); }
WEAK void DMA1_Channel1_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel2_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel3_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel4_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel5_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel6_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA1_Channel7_IRQHandler  (void) { _IllegalException(); }
WEAK void ADC1_2_IRQHandler         (void) { _IllegalException(); }
WEAK void USB_HP_CAN1_TX_IRQHandler (void) { _IllegalException(); }
WEAK void USB_LP_CAN1_RX0_IRQHandler(void) { _IllegalException(); }
WEAK void CAN1_RX1_IRQHandler       (void) { _IllegalException(); }
WEAK void CAN1_SCE_IRQHandler       (void) { _IllegalException(); }
WEAK void EXTI9_5_IRQHandler        (void) { _IllegalException(); }
WEAK void TIM1_BRK_IRQHandler       (void) { _IllegalException(); }
WEAK void TIM1_UP_IRQHandler        (void) { _IllegalException(); }
WEAK void TIM1_TRG_COM_IRQHandler   (void) { _IllegalException(); }
WEAK void TIM1_CC_IRQHandler        (void) { _IllegalException(); }
WEAK void TIM2_IRQHandler           (void) { _IllegalException(); }
WEAK void TIM3_IRQHandler           (void) { _IllegalException(); }
WEAK void TIM4_IRQHandler           (void) { _IllegalException(); }
WEAK void I2C1_EV_IRQHandler        (void) { _IllegalException(); }
WEAK void I2C1_ER_IRQHandler        (void) { _IllegalException(); }
WEAK void I2C2_EV_IRQHandler        (void) { _IllegalException(); }
WEAK void I2C2_ER_IRQHandler        (void) { _IllegalException(); }
WEAK void SPI1_IRQHandler           (void) { _IllegalException(); }
WEAK void SPI2_IRQHandler           (void) { _IllegalException(); }
WEAK void USART1_IRQHandler         (void) { _IllegalException(); }
WEAK void USART2_IRQHandler         (void) { _IllegalException(); }
WEAK void USART3_IRQHandler         (void) { _IllegalException(); }
WEAK void EXTI15_10_IRQHandler      (void) { _IllegalException(); }
WEAK void RTCAlarm_IRQHandler       (void) { _IllegalException(); }
WEAK void USBWakeUp_IRQHandler      (void) { _IllegalException(); }
WEAK void TIM8_BRK_IRQHandler       (void) { _IllegalException(); }
WEAK void TIM8_UP_IRQHandler        (void) { _IllegalException(); }
WEAK void TIM8_TRG_COM_IRQHandler   (void) { _IllegalException(); }
WEAK void TIM8_CC_IRQHandler        (void) { _IllegalException(); }
WEAK void ADC3_IRQHandler           (void) { _IllegalException(); }
WEAK void FSMC_IRQHandler           (void) { _IllegalException(); }
WEAK void SDIO_IRQHandler           (void) { _IllegalException(); }
WEAK void TIM5_IRQHandler           (void) { _IllegalException(); }
WEAK void SPI3_IRQHandler           (void) { _IllegalException(); }
WEAK void UART4_IRQHandler          (void) { _IllegalException(); }
WEAK void UART5_IRQHandler          (void) { _IllegalException(); }
WEAK void TIM6_IRQHandler           (void) { _IllegalException(); }
WEAK void TIM7_IRQHandler           (void) { _IllegalException(); }
WEAK void DMA2_Channel1_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA2_Channel2_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA2_Channel3_IRQHandler  (void) { _IllegalException(); }
WEAK void DMA2_Channel4_5_IRQHandler(void) { _IllegalException(); }

/*********************************************************************
*
*       Global code
*
**********************************************************************
*/

/*********************************************************************
*
*       Exception vector table
*/
#ifdef __ICCARM__
  #if (__VER__ < 500)
    #pragma location = "INTVEC"
  #else
    #pragma location = ".intvec"
  #endif  // #if (__VER__ < 500)
#endif    // #ifdef __ICCARM__
#ifdef __CC_ARM
  #pragma arm section rodata = "INTVEC"
#endif

/****** STM32F10x Vector Table entries ******************************/
const intvec_elem __vector_table[] =
{
#ifdef __ICCARM__
  { .__ptr = __sfe( "CSTACK" ) },
#if (__VER__ < 500)
  __program_start,
#else
  __iar_program_start,
#endif  // #if (__VER__ >= 500)
#endif  // #ifdef __ICCARM__
#ifdef __CC_ARM
  (intfunc) &Image$$CSTACK$$ZI$$Limit,
  __main,
#endif

  /**** System exceptions *****************/

  NMI_Handler,
  HardFault_Handler,
  MemManage_Handler,
  BusFault_Handler,
  UsageFault_Handler,
  0, 0, 0, 0,            /* Reserved */
  SVC_Handler,
  DebugMon_Handler,
  0,                     /* Reserved */
  PendSV_Handler,        /* Required for the OS ! */
  SysTick_Handler,       /* Required for the OS ! */

  /**** Peripheral exceptions / interrupts */

  WWDG_IRQHandler,              /* 0  Window Watchdog */
  PVD_IRQHandler,               /* 1  PVD through EXTI Line detect */
  TAMPER_IRQHandler,            /* 2  Tamper */
  RTC_IRQHandler,               /* 3  RTC */
  FLASH_IRQHandler,             /* 4  Flash */
  RCC_IRQHandler,               /* 5  RCC */
  EXTI0_IRQHandler,             /* 6  EXTI Line 0 */
  EXTI1_IRQHandler,             /* 7  EXTI Line 1 */
  EXTI2_IRQHandler,             /* 8  EXTI Line 2 */
  EXTI3_IRQHandler,             /* 9  EXTI Line 3 */
  EXTI4_IRQHandler,             /* 10 EXTI Line 4 */
  DMA1_Channel1_IRQHandler,     /* 11 DMA1 Channel 1 */
  DMA1_Channel2_IRQHandler,     /* 12 DMA1 Channel 2 */
  DMA1_Channel3_IRQHandler,     /* 13 DMA1 Channel 3 */
  DMA1_Channel4_IRQHandler,     /* 14 DMA1 Channel 4 */
  DMA1_Channel5_IRQHandler,     /* 15 DMA1 Channel 5 */
  DMA1_Channel6_IRQHandler,     /* 16 DMA1 Channel 6 */
  DMA1_Channel7_IRQHandler,     /* 17 DMA1 Channel 7 */
  ADC1_2_IRQHandler,            /* 18 ADC1 & ADC2 */
  USB_HP_CAN1_TX_IRQHandler,    /* 19 USB High Priority or CAN1 TX */
  USB_LP_CAN1_RX0_IRQHandler,   /* 20 USB Low  Priority or CAN1 RX0 */
  CAN1_RX1_IRQHandler,          /* 21 CAN1 RX1 */
  CAN1_SCE_IRQHandler,          /* 22 CAN1 SCE */
  EXTI9_5_IRQHandler,           /* 23 EXTI Line 9..5 */
  TIM1_BRK_IRQHandler,          /* 24 TIM1 Break */
  TIM1_UP_IRQHandler,           /* 25 TIM1 Update */
  TIM1_TRG_COM_IRQHandler,      /* 26 TIM1 Trigger and Commutation */
  TIM1_CC_IRQHandler,           /* 27 TIM1 Capture Compare */
  TIM2_IRQHandler,              /* 28 TIM2 */
  TIM3_IRQHandler,              /* 29 TIM3 */
  TIM4_IRQHandler,              /* 30 TIM4 */
  I2C1_EV_IRQHandler,           /* 31 I2C1 Event */
  I2C1_ER_IRQHandler,           /* 32 I2C1 Error */
  I2C2_EV_IRQHandler,           /* 33 I2C2 Event */
  I2C2_ER_IRQHandler,           /* 34 I2C2 Error */
  SPI1_IRQHandler,              /* 35 SPI1 */
  SPI2_IRQHandler,              /* 36 SPI2 */
  USART1_IRQHandler,            /* 37 USART1 */
  USART2_IRQHandler,            /* 38 USART2 */
  USART3_IRQHandler,            /* 39 USART3 */
  EXTI15_10_IRQHandler,         /* 40 EXTI Line 15..10 */
  RTCAlarm_IRQHandler,          /* 41 RTC Alarm through EXTI Line */
  USBWakeUp_IRQHandler,         /* 42 USB Wakeup from suspend */
  TIM8_BRK_IRQHandler,          /* 43 TIM8 Break */
  TIM8_UP_IRQHandler,           /* 44 TIM8 Update */
  TIM8_TRG_COM_IRQHandler,      /* 45 TIM8 Trigger and Commutation */
  TIM8_CC_IRQHandler,           /* 46 TIM8 Capture Compare */
  ADC3_IRQHandler,              /* 47 ADC3 */
  FSMC_IRQHandler,              /* 48 FSMC */
  SDIO_IRQHandler,              /* 49 SDIO */
  TIM5_IRQHandler,              /* 50 TIM5 */
  SPI3_IRQHandler,              /* 51 SPI3 */
  UART4_IRQHandler,             /* 52 UART4 */
  UART5_IRQHandler,             /* 53 UART5 */
  TIM6_IRQHandler,              /* 54 TIM6 */
  TIM7_IRQHandler,              /* 55 TIM7 */
  DMA2_Channel1_IRQHandler,     /* 56 DMA2 Channel1 */
  DMA2_Channel2_IRQHandler,     /* 57 DMA2 Channel2 */
  DMA2_Channel3_IRQHandler,     /* 58 DMA2 Channel3 */
  DMA2_Channel4_5_IRQHandler    /* 59 DMA2 Channel4 & Channel5 */
};

#ifdef __cplusplus
  }
#endif

/****** End Of File *************************************************/
