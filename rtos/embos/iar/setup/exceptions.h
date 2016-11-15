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
File    : exceptions.h
Purpose : CMSIS compatible definition of exception handler for AT91SAM3S
          To be included in all sources which implement
          or reference exception/irq handler
--------  END-OF-HEADER  ---------------------------------------------
*/

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Define WEAK attribute                                            */
/* All default exception handlers are implemented weak,             */
/* so they can be replaced by the application.                      */
#if defined   ( __CC_ARM   )
  #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ )
  #define WEAK __weak
#elif defined (  __GNUC__  )
  #define WEAK __attribute__ ((weak))
#endif

/********************************************************************/

/****** CM3 internal exceptions *************************************/

extern WEAK void NMI_Handler(void);
extern WEAK void HardFault_Handler(void);
extern WEAK void MemManage_Handler(void);
extern WEAK void BusFault_Handler(void);
extern WEAK void UsageFault_Handler(void);
extern WEAK void SVC_Handler(void);
extern WEAK void DebugMon_Handler(void);
extern WEAK void PendSV_Handler(void);
extern WEAK void SysTick_Handler(void);

/****** Controller specific peripheral interrupts *******************/

extern WEAK void WWDG_IRQHandler(void);
extern WEAK void PVD_IRQHandler(void);
extern WEAK void TAMPER_IRQHandler(void);
extern WEAK void RTC_IRQHandler(void);
extern WEAK void FLASH_IRQHandler(void);
extern WEAK void RCC_IRQHandler(void);
extern WEAK void EXTI0_IRQHandler(void);
extern WEAK void EXTI1_IRQHandler(void);
extern WEAK void EXTI2_IRQHandler(void);
extern WEAK void EXTI3_IRQHandler(void);
extern WEAK void EXTI4_IRQHandler(void);
extern WEAK void DMA1_Channel1_IRQHandler(void);
extern WEAK void DMA1_Channel2_IRQHandler(void);
extern WEAK void DMA1_Channel3_IRQHandler(void);
extern WEAK void DMA1_Channel4_IRQHandler(void);
extern WEAK void DMA1_Channel5_IRQHandler(void);
extern WEAK void DMA1_Channel6_IRQHandler(void);
extern WEAK void DMA1_Channel7_IRQHandler(void);
extern WEAK void ADC1_2_IRQHandler(void);
extern WEAK void USB_HP_CAN1_TX_IRQHandler(void);
extern WEAK void USB_LP_CAN1_RX0_IRQHandler(void);
extern WEAK void CAN1_RX1_IRQHandler(void);
extern WEAK void CAN1_SCE_IRQHandler(void);
extern WEAK void EXTI9_5_IRQHandler(void);
extern WEAK void TIM1_BRK_IRQHandler(void);
extern WEAK void TIM1_UP_IRQHandler(void);
extern WEAK void TIM1_TRG_COM_IRQHandler(void);
extern WEAK void TIM1_CC_IRQHandler(void);
extern WEAK void TIM2_IRQHandler(void);
extern WEAK void TIM3_IRQHandler(void);
extern WEAK void TIM4_IRQHandler(void);
extern WEAK void I2C1_EV_IRQHandler(void);
extern WEAK void I2C1_ER_IRQHandler(void);
extern WEAK void I2C2_EV_IRQHandler(void);
extern WEAK void I2C2_ER_IRQHandler(void);
extern WEAK void SPI1_IRQHandler(void);
extern WEAK void SPI2_IRQHandler(void);
extern WEAK void USART1_IRQHandler(void);
extern WEAK void USART2_IRQHandler(void);
extern WEAK void USART3_IRQHandler(void);
extern WEAK void EXTI15_10_IRQHandler(void);
extern WEAK void RTCAlarm_IRQHandler(void);
extern WEAK void USBWakeUp_IRQHandler(void);
extern WEAK void TIM8_BRK_IRQHandler(void);
extern WEAK void TIM8_UP_IRQHandler(void);
extern WEAK void TIM8_TRG_COM_IRQHandler(void);
extern WEAK void TIM8_CC_IRQHandler(void);
extern WEAK void ADC3_IRQHandler(void);
extern WEAK void FSMC_IRQHandler(void);
extern WEAK void SDIO_IRQHandler(void);
extern WEAK void TIM5_IRQHandler(void);
extern WEAK void SPI3_IRQHandler(void);
extern WEAK void UART4_IRQHandler(void);
extern WEAK void UART5_IRQHandler(void);
extern WEAK void TIM6_IRQHandler(void);
extern WEAK void TIM7_IRQHandler(void);
extern WEAK void DMA2_Channel1_IRQHandler(void);
extern WEAK void DMA2_Channel2_IRQHandler(void);
extern WEAK void DMA2_Channel3_IRQHandler(void);
extern WEAK void DMA2_Channel4_5_IRQHandler(void);

#ifdef __cplusplus
  }
#endif

/********************************************************************/

#endif                                  /* Avoid multiple inclusion */

/*************************** End of file ****************************/
