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
File    : RTOSInit.c
Purpose : RTOSInit for STM32F103 (MB525 & MB672). Initializes and
          handles the hardware for the OS as far as required by the OS.
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"
#ifdef __ICCARM__  // IAR
  #include "exceptions.h"           /* CMSIS compatible irq interface */
#endif

/*********************************************************************
*
*       Configuration
*
*********************************************************************/

/*********************************************************************
*
*       Clock frequency settings
*/
#ifndef   OS_FSYS                   /* CPU main clock frequency      */
  #define OS_FSYS 72000000uL
#endif

#ifndef   OS_PCLK_TIMER             /* Peripheral clock for timer   */
  #define OS_PCLK_TIMER OS_FSYS     /* May vary from CPU clock      */
#endif                              /* depending on CPU             */

#ifndef   OS_PCLK_UART              /* Peripheral clock for uart    */
  #define OS_PCLK_UART OS_FSYS      /* May vary from CPU clock      */
#endif                              /* depending on CPU             */

#ifndef   OS_TICK_FREQ
  #define OS_TICK_FREQ (1000)
#endif

#ifndef   OS_USE_VARINTTABLE        /* The interrupt vector table   */
  #define OS_USE_VARINTTABLE (0)    /* may be located in RAM        */
#endif

#define OS_Systick SysTick_Handler  /* Map OS systick handler to CMSIS compatible handler name */

/*********************************************************************
*
*       Configuration of communication to OSView
*/
#ifndef   OS_VIEW_ENABLE            // Global enable of communication
  #define OS_VIEW_ENABLE    (0)     // Default: on
#endif

#ifndef   OS_VIEW_USE_UART          // If set, UART will be used for communication
  #define OS_VIEW_USE_UART (0)      // Default: 0 => memory access is used
#endif                              // if OS_VIEW_ENABLE is on

/*********************************************************************
*
*       UART settings for OSView
*       If you do not want (or can not due to hardware limitations)
*       to dedicate a UART to embOSView, please define it to be -1
*       Currently the standard code enables OS_UART 0 per default.
*       OS_UART=0: UART0 (CN6) on MB525/672 evalboard
*       OS_UART=1: UART1 (CN5) on MB525/672 evalboard
*/
#ifndef   OS_UART
  #define OS_UART (-1)
#endif

#ifndef   OS_BAUDRATE
  #define OS_BAUDRATE (38400)
#endif

/****** End of configuration settings *******************************/

#ifndef   DEBUG
  #define DEBUG (0)
#endif

/*********************************************************************
*
*       JLINKMEM and UART settings for OSView
*
*       Automatically generated from configuration settings
*/
#define OS_USE_JLINKMEM   (OS_VIEW_ENABLE && (OS_VIEW_USE_UART == 0))

#define OS_UART_USED      (OS_VIEW_ENABLE && (OS_VIEW_USE_UART != 0) && ((OS_UART == 0) || (OS_UART == 1)))

#if OS_USE_JLINKMEM
  #include "JLINKMEM.h"
#endif

/*********************************************************************
*
*       Vector table
*/
#ifdef __ICCARM__
  #define __Vectors    __vector_table
#else
  extern unsigned char __Vectors;
#endif

/*********************************************************************
*
*       Local defines (sfrs used in RTOSInit.c)
*
**********************************************************************
*/
#define SYS_INT_CTRL_STATE            (*(volatile OS_U32*)(0xE000ED04))
#define SYS_PENDSTSET                 26

#define SYSPRI1_ADDR                  (0xE000ED18)
#define SYSHND_CTRL_ADDR              (0xE000ED24)    // System Handler Control and State
#define SYSHND_CTRL                   (*(volatile OS_U32*) (SYSHND_CTRL_ADDR))

#define NVIC_SYS_HND_CTRL_MEM         (0x00010000uL)  // Mem manage fault enable
#define NVIC_SYS_HND_CTRL_BUS         (0x00020000uL)  // Bus fault enable
#define NVIC_SYS_HND_CTRL_USAGE       (0x00040000uL)  // Usage fault enable

#define NVIC_PRIOBASE_ADDR            (0xE000E400)
#define NVIC_ENABLE_ADDR              (0xE000E100)
#define NVIC_DISABLE_ADDR             (0xE000E180)
#define NVIC_VTOREG_ADDR              (0xE000ED08)
#define NVIC_VTOR                     (*(volatile unsigned long *) (0xE000ED08))

#define NUM_INTERRUPTS                (16+60)

#define AFIO_BASE_ADDR                (0x40010000)
#define AFIO_MAPR                     (*(volatile OS_U32*)(AFIO_BASE_ADDR + 0x04))

#define PERIPH_BASE_ADDR              ((OS_U32)0x40000000)
#define APB1PERIPH_BASE_ADDR          (PERIPH_BASE_ADDR)
#define APB2PERIPH_BASE_ADDR          (PERIPH_BASE_ADDR + 0x10000)
#define AHBPERIPH_BASE_ADDR           (PERIPH_BASE_ADDR + 0x20000)

#define RCC_BASE_ADDR                 (AHBPERIPH_BASE_ADDR + 0x1000)
#define RCC_APB2ENR                   (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x18))

#define RCC_CR                        (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x00))
#define RCC_CFGR                      (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x04))
#define RCC_CIR                       (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x08))
#define RCC_APB2RSTR                  (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x0C))
#define RCC_APB1RSTR                  (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x10))
#define RCC_AHBENR                    (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x14))
#define RCC_APB1ENR                   (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x1C))
#define RCC_BDCR                      (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x20))
#define RCC_CSR                       (*(volatile OS_U32*)(RCC_BASE_ADDR + 0x24))

#define SYS_TICK_BASE_ADDR            (0xE000E010)
#define SYS_TICK_CONTROL              (*(volatile OS_U32*)(SYS_TICK_BASE_ADDR + 0x00))
#define SYS_TICK_RELOAD               (*(volatile OS_U32*)(SYS_TICK_BASE_ADDR + 0x04))
#define SYS_TICK_VALUE                (*(volatile OS_U32*)(SYS_TICK_BASE_ADDR + 0x08))
#define SYS_TICK_CALIBRATION          (*(volatile OS_U32*)(SYS_TICK_BASE_ADDR + 0x0C))
#define SYS_TICK_ENABLE_BIT           (0)
#define SYS_TICK_INT_ENABLE_BIT       (1)
#define SYS_TICK_CLK_SOURCE_BIT       (2)

#define CR_PLLON_BB                   (*(volatile OS_U32*)(0x42420060))
#define CR_HSEBYP_Reset               ((OS_U32)0xFFFBFFFF)
#define CR_HSEBYP_Set                 ((OS_U32)0x00040000)
#define CR_HSEON_Reset                ((OS_U32)0xFFFEFFFF)

#define CR_HSEON_Set                  ((OS_U32)0x00010000)
#define CR_HSITRIM_Mask               ((OS_U32)0xFFFFFF07)
#define FLASH_Latency_2               ((OS_U32)0x00000002)
#define ACR_LATENCY_Mask              ((OS_U32)0x00000038)
#define ACR_PRFTBE_Mask               ((OS_U32)0xFFFFFFEF)
#define FLASH_PrefetchBuffer_Enable   ((OS_U32)0x00000010)
#define CFGR_HPRE_Reset_Mask          ((OS_U32)0xFFFFFF0F)
#define RCC_SYSCLK_Div1               ((OS_U32)0x00000000)
#define CFGR_PPRE2_Reset_Mask         ((OS_U32)0xFFFFC7FF)
#define RCC_HCLK_Div1                 ((OS_U32)0x00000000)
#define RCC_HCLK_Div2                 ((OS_U32)0x00000400)
#define CFGR_PPRE1_Reset_Mask         ((OS_U32)0xFFFFF8FF)
#define CFGR_PLL_Mask                 ((OS_U32)0xFFC0FFFF)
#define PERIPH_BB_BASE                ((OS_U32)0x42000000)
#define PLLON_BitNumber               (0x18)
#define CFGR_SW_Mask                  ((OS_U32)0xFFFFFFFC)
#define RCC_SYSCLKSource_PLLCLK       ((OS_U32)0x00000002)
#define CFGR_SWS_Mask                 ((OS_U32)0x0000000C)
#define RCC_PLLSource_HSE_Div1        ((OS_U32)0x00010000)
#define RCC_PLLMul_9                  ((OS_U32)0x001C0000)

#define FLASH_BASE_ADDR               ((OS_U32)0x40022000)
#define FLASH_ACR                     (*(volatile OS_U32*)(FLASH_BASE_ADDR + 0x00))

/*********************************************************************
*
* The following can be used as as arguments for the PLL activation
* if required in __low_level_init()
*
**********************************************************************
*/
#define SYSCTRL_SYSDIV_1              (0x07800000)  // Processor clock is osc/pll /1
#define SYSCTRL_SYSDIV_4              (0x01C00000)  // Processor clock is osc/pll /4
#define SYSCTRL_SYSDIV_10             (0x04C00000)  // Processor clock is osc/pll /10
#define SYSCTRL_USE_PLL               (0x00000000)  // System clock is the PLL clock
#define SYSCTRL_USE_OSC               (0x00003800)  // System clock is the osc clock
#define SYSCTRL_XTAL_6MHZ             (0x000002C0)  // External crystal is 6MHz
#define SYSCTRL_XTAL_8MHZ             (0x00000380)  // External crystal is 8MHz
#define SYSCTRL_OSC_MAIN              (0x00000000)  // Oscillator source is main osc

/*****  Interrupt ID numbers ****************************************/

#define ISR_ID_MPU                    (4)                // MPU fault
#define ISR_ID_BUS                    (5)                // Bus fault
#define ISR_ID_USAGE                  (6)                // Usage fault
#define ISR_ID_SYSTICK                (15)               // System Tick
#define ISR_ID_USART1                 (53)               // USART1
#define ISR_ID_USART2                 (54)               // USART2
  
#define ISR_ID_SPI2                   (36)               // SPI2

#define OS_ISR_ID_TICK                ISR_ID_SYSTICK     // We use Sys-Timer

/****** OS timer configuration **************************************/

#define OS_TIMER_RELOAD           (OS_PCLK_TIMER / OS_TICK_FREQ - 1)
#if (OS_TIMER_RELOAD >= 0x100000000)
  #error "Systick can not be used, please check configuration"
#endif

/****** MAP UART initialization function ****************************/

#if  (OS_UART_USED || OS_USE_JLINKMEM)
  #define OS_COM_INIT() OS_COM_Init()
#else
  #define OS_COM_INIT()
#endif

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

#if OS_USE_JLINKMEM
  // Size of the communication buffer for JLINKMEM
const OS_U32 OS_JLINKMEM_BufferSize = 32;
#else
const OS_U32 OS_JLINKMEM_BufferSize = 0;     // Communication not used
#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       OS_Systick
*
* Function description
*   This is the code that gets called when the processor receives a
*   _SysTick exception. SysTick is used as OS timer tick.
*
* NOTES:
*   (1) It has to be inserted in the interrupt vector table, if RAM
*       vectors are not used. Therefore it is declared public
*/
void OS_Systick(void) {
  OS_EnterNestableInterrupt();
  OS_TICK_Handle();
#if OS_USE_JLINKMEM
  JLINKMEM_Process();
#endif
  OS_LeaveNestableInterrupt();
}

/*********************************************************************
*
*       OS_InitHW()
*
*       Initialize the hardware (timer) required for the OS to run.
*       May be modified, if an other timer should be used
*/
void OS_InitHW(void) {
  OS_U8  TickPrio;
  OS_U32 tmpreg;

  OS_IncDI();                            // Initially disable interrupts
  RCC_APB2RSTR = 0x00000000;             // Disable APB2 Peripheral Reset
  RCC_APB1RSTR = 0x00000000;             // Disable APB1 Peripheral Reset
  RCC_AHBENR   = 0x00000014;             // FLITF and SRAM Clock ON
  RCC_APB2ENR  = 0x00000000;             // Disable APB2 Peripheral Clock
  RCC_APB1ENR  = 0x00000000;             // Disable APB1 Peripheral Clock
  RCC_CR      |= (OS_U32)0x00000001;     // Set HSION bit
  RCC_CFGR    &= 0xF8FF0000;             // Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits
  RCC_CR      &= 0xFEF6FFFF;             // Reset HSEON, CSSON and PLLON bits
  RCC_CR      &= 0xFFFBFFFF;             // Reset HSEBYP bit
  RCC_CFGR    &= 0xFF80FFFF;             // Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits
  RCC_CIR      = 0x00000000;             // Disable all interrupts
  RCC_CR      &= CR_HSEON_Reset;         // Reset HSEON bit
  RCC_CR      &= CR_HSEBYP_Reset;        // Reset HSEBYP bit
  RCC_CR      |= CR_HSEON_Set;           // Configure HSE (RCC_HSE_OFF is already covered by the code section above
  while ((RCC_CR & (1uL << 17) == 0));   // Wait till HSE is ready (The RCC_CR_HSERDY_BIT (Bit 17) is 1 if the HSE is running.)
  FLASH_ACR   &= ACR_LATENCY_Mask;       // Flash 2 wait state and Prefetch enable
  FLASH_ACR   |= FLASH_Latency_2;
  FLASH_ACR   &= ACR_PRFTBE_Mask;        // Enable or disable the Prefetch Buffer
  FLASH_ACR   |= FLASH_PrefetchBuffer_Enable;
  RCC_CFGR    &= CFGR_HPRE_Reset_Mask;   // HCLK = SYSCLK
  RCC_CFGR    |= RCC_SYSCLK_Div1;
  RCC_CFGR    &= CFGR_PPRE2_Reset_Mask;  // PCLK2 = HCLK
  RCC_CFGR     = (RCC_HCLK_Div1 << 3);
  RCC_CFGR    &= CFGR_PPRE1_Reset_Mask;  // PCLK1 = HCLK/2
  RCC_CFGR     = RCC_HCLK_Div2;
  tmpreg       = RCC_CFGR;               // PLLCLK = 8MHz * 9 = 72 MHz */
  tmpreg      &= CFGR_PLL_Mask;          // Clear PLLSRC, PLLXTPRE and PLLMUL[21:18] bits
  tmpreg      |= RCC_PLLSource_HSE_Div1 | RCC_PLLMul_9;// Set the PLL configuration bits
  RCC_CFGR     = tmpreg;                 // Store the new value
  CR_PLLON_BB  = (OS_U32)0x01;           // Enable PLL
  while ((RCC_CR & (1uL << 25) == 0));   // Wait till PLL is ready
  tmpreg   = RCC_CFGR;                   // Select PLL as system clock source
  tmpreg  &= CFGR_SW_Mask;               // Clear SW[1:0] bits
  tmpreg  |= RCC_SYSCLKSource_PLLCLK;    // Set SW[1:0] bits according to RCC_SYSCLKSource value
  RCC_CFGR = tmpreg;                     // Store the new value
  while ((RCC_CFGR & CFGR_SWS_Mask) != 0x08); // Wait till PLL is used as system clock source

  //
  // Initialize NVIC vector base address. Might be necessary for RAM targets or application not running from 0
  //
  NVIC_VTOR = (OS_U32)&__Vectors;
  //
  // Initialize OS timer, clock soure = core clock
  //
  SYS_TICK_RELOAD  = OS_TIMER_RELOAD;
  SYS_TICK_CONTROL = (1uL << SYS_TICK_ENABLE_BIT) | (1uL << SYS_TICK_CLK_SOURCE_BIT);
  //
  // Install Systick Timer Handler and enable timer interrupt
  //
  OS_ARM_InstallISRHandler(OS_ISR_ID_TICK, (OS_ISR_HANDLER*)OS_Systick);
  //
  // Set the interrupt priority for the system timer to 2nd lowest level to ensure the timer can preempt PendSV handler
  //
  OS_ARM_ISRSetPrio(OS_ISR_ID_TICK, 0xFF);               // Set to lowest level, ALL BITS set
  TickPrio  = OS_ARM_ISRSetPrio(OS_ISR_ID_TICK, 0xFF);   // Read priority back to examine relevant preemption-level-bits
  TickPrio -= 1;                                         // Set to low preemption level, 1 level higher than lowest
  OS_ARM_ISRSetPrio(OS_ISR_ID_TICK, TickPrio);
  OS_ARM_EnableISR(OS_ISR_ID_TICK);
  OS_COM_INIT();
  OS_DecRI();
}

/*********************************************************************
*
*       Idle loop  (OS_Idle)
*
*       Please note:
*       This is basically the "core" of the idle loop.
*       This core loop can be changed, but:
*       The idle loop does not have a stack of its own, therefore no
*       functionality should be implemented that relies on the stack
*       to be preserved. However, a simple program loop can be programmed
*       (like toggeling an output or incrementing a counter)
*/
void OS_Idle(void) {     // Idle loop: No task is ready to execute
  while (1) {
    #if ((OS_USE_JLINKMEM == 0) && (DEBUG == 0))     // Enter CPU halt mode when not in DEBUG build and J-Link communication not used
      #ifdef __ICCARM__  // IAR
        __asm(" wfi");
      #endif
      #ifdef __CC_ARM    // KEIL
        __wfi();
      #endif
    #endif
  }
}

/*********************************************************************
*
*       OS_GetTime_Cycles()
*
*       This routine is required for high
*       resolution time measurement functions.
*       It returns the system time in timer clock cycles.
*/
OS_U32 OS_GetTime_Cycles(void) {
  unsigned int t_cnt;
  OS_U32 time;
  time  = OS_Time;
  t_cnt = (OS_PCLK_TIMER/1000) - SYS_TICK_VALUE;
  if (SYS_INT_CTRL_STATE & (1uL << SYS_PENDSTSET)) {         // Missed a counter interrupt?
    t_cnt = (OS_PCLK_TIMER/1000) - SYS_TICK_VALUE;           // Adjust result
    time++;
  }
  return (OS_PCLK_TIMER/1000) * time + t_cnt;
}

/*********************************************************************
*
*       OS_ConvertCycles2us
*
*       Convert Cycles into micro seconds.
*
*       If your clock frequency is not a multiple of 1 MHz,
*       you may have to modify this routine in order to get proper
*       diagonstics.
*
*       This routine is required for profiling or high resolution time
*       measurement only. It does not affect operation of the OS.
*/
OS_U32 OS_ConvertCycles2us(OS_U32 Cycles) {
  return Cycles/(OS_PCLK_TIMER/1000000);
}

#if OS_UART_USED
/*********************************************************************
*
*       Communication for OSView via UART (optional)
*
**********************************************************************
*/
#if (OS_UART == 0)
  #define OS_COM_IsrHandler USART1_IRQHandler
  #define USART_BASE_ADDR   (0x40013800)
  #define OS_BRR_VALUE      (OS_PCLK_UART / OS_BAUDRATE)
#elif (OS_UART == 1)
  #define OS_COM_IsrHandler USART2_IRQHandler
  #define USART_BASE_ADDR   (0x40004400)
  #if (OS_PCLK_UART > 36000000uL)
    //
    // BUS is limited to 36MHz, assume, clock frequency is divided by 2
    //
    #define OS_BRR_VALUE     (OS_PCLK_UART / OS_BAUDRATE / 2)
  #else
    #define OS_BRR_VALUE     (OS_PCLK_UART / OS_BAUDRATE)
  #endif
#endif
#define USART_SR             (*(volatile OS_U32*)(USART_BASE_ADDR + 0x00))
#define USART_DR             (*(volatile OS_U32*)(USART_BASE_ADDR + 0x04))
#define USART_BRR            (*(volatile OS_U32*)(USART_BASE_ADDR + 0x08))
#define USART_CR1            (*(volatile OS_U32*)(USART_BASE_ADDR + 0x0C))
#define USART_CR2            (*(volatile OS_U32*)(USART_BASE_ADDR + 0x10))
#define USART_CR3            (*(volatile OS_U32*)(USART_BASE_ADDR + 0x14))
#define USART_GTPR           (*(volatile OS_U32*)(USART_BASE_ADDR + 0x18))
#define RCC_USART1EN         ((OS_U32)0x00004004)
#ifdef UART2_NOT_REMAPPED
  #define RCC_USART2EN       ((OS_U32)0x00000014)
#else  // UART 2 is on Port D
  #define RCC_USART2EN       ((OS_U32)0x00000024)
#endif

#define GPIOA_CRL            (*(volatile OS_U32*)(0x40010800))
#define GPIOA_CRH            (*(volatile OS_U32*)(0x40010804))
#define GPIOD_CRL            (*(volatile OS_U32*)(0x40011400))


#define US_RXRDY             (0x20)   // RXNE
#define USART_RX_ERROR_FLAGS (0x0F)   // ORE/NE/FE/PE
#define US_TXEMPTY           (0x80)   // TXE

/*********************************************************************
*
*       OS_COM_IsrHandler
*
* Function description
*   The communication interrupt handler for UART communication
*   to embOSView.
*
* NOTES:
*   (1) It has to be inserted in the interrupt vector table, if RAM
*       vectors are not used. Therefore is is declared public
*/
void OS_COM_IsrHandler(void) {
  volatile OS_U32 Dummy;
  int UsartStatus;
  OS_EnterNestableInterrupt();
  UsartStatus = USART_SR;                        // Examine status register
  do {
    if (UsartStatus & US_RXRDY) {                // Data received?
      if (UsartStatus & USART_RX_ERROR_FLAGS) {  // Any error ?
        Dummy = USART_DR;                        // => Discard data
      } else {
        OS_OnRx(USART_DR);                       // Process actual byte
      }
    }
    UsartStatus = USART_SR;                      // Examine current status
  } while (UsartStatus & US_RXRDY);
  if (UsartStatus & US_TXEMPTY) {
    if (OS_OnTx()) {                             // No more characters to send ?
      USART_CR1 &= ~0x40uL;                      // Disable further tx interrupts
    }
  }
  OS_LeaveNestableInterrupt();
}

/*********************************************************************
*
*       OS_COM_Send1()
*
*       Sends one character via UART. Never call this from your application
*/
void OS_COM_Send1(OS_U8 c) {
  USART_DR   = (c & (OS_U16)0x01FF);
  USART_CR1 |= 0x40;                           // Enable tx interrupt
}

/*********************************************************************
*
*       OS_COM_Init()
*
*       Initialize the selected UART
*/
#if (OS_UART == 0)
void OS_COM_Init(void) {
  OS_U8 Priority;

  OS_IncDI();
  RCC_APB2ENR |= RCC_USART1EN;  // Enable GPIO port used for USART and USART clock
  GPIOA_CRH    = (GPIOA_CRH & 0xFFFFF00F) | 0x000004B0;
  USART_CR2    = 0x200;
  USART_CR1    = 0x2C;
  USART_CR3    = 0x00;
  USART_BRR    = OS_BRR_VALUE;
  USART_CR1   |= 0x2000;         // Enable uart
  //
  // Install USART Handler with preemtion level one above lowest level to ensure communication during PendSV
  //
  OS_ARM_InstallISRHandler(ISR_ID_USART1, (OS_ISR_HANDLER*)OS_COM_IsrHandler);
  OS_ARM_ISRSetPrio(ISR_ID_USART1, 0xFF);               // Set lowest Priority, ALL BITS set
  Priority  = OS_ARM_ISRSetPrio(ISR_ID_USART1, 0xFF);   // Read back priority to examine preemption level bits
  Priority -= 1;                                        // Set priority one level above lowest priority
  OS_ARM_ISRSetPrio(ISR_ID_USART1, Priority);
  OS_ARM_EnableISR(ISR_ID_USART1);
  OS_DecRI();
}

#elif (OS_UART == 1)
void OS_COM_Init(void) {
  OS_U8 Priority;

  OS_IncDI();
  RCC_APB1ENR |= (1uL << 17);               // Enable USART2 clock
  RCC_APB2ENR |= RCC_USART2EN  | (1uL << 0);// Enable GPIO port needed for USART, enable alternate function
#ifndef UART2_NOT_REMAPPED
  AFIO_MAPR   |= (1uL << 3);                // Remap USART to Port D
  GPIOD_CRL    = (GPIOD_CRL & 0xF00FFFFF) | 0x04B00000;
#else
  GPIOA_CRL    = (GPIOA_CRL & 0xFFFF00FF) | 0x00004B00;
#endif
  USART_CR2    = 0x200;
  USART_CR1    = 0x2C;
  USART_CR3    = 0x00;
  USART_BRR    = OS_BRR_VALUE;
  USART_CR1   |= 0x2000;         // Enable uart
  //
  // Install USART2 Handler with preemtion level one above lowest level to ensure communication
  //
  OS_ARM_InstallISRHandler(ISR_ID_USART2, (OS_ISR_HANDLER*)OS_COM_IsrHandler);
  OS_ARM_ISRSetPrio(ISR_ID_USART2, 0xFF);               // Set lowest Priority, ALL BITS set
  Priority  = OS_ARM_ISRSetPrio(ISR_ID_USART2, 0xFF);   // Read back priority to examine preemption level bits
  Priority -= 1;                                        // Set priority one level above lowest priority
  OS_ARM_ISRSetPrio(ISR_ID_USART2, Priority);
  OS_ARM_EnableISR(ISR_ID_USART2);
  OS_DecRI();
}
#endif

#elif OS_USE_JLINKMEM

/*********************************************************************
*
*       _JLINKMEM_OnRx()
*/
static void _JLINKMEM_OnRx(OS_U8 Data) {
  OS_OnRx(Data);
}

/*********************************************************************
*
*       _JLINKMEM_OnTx()
*/
static void _JLINKMEM_OnTx(void) {
  OS_OnTx();
}

/*********************************************************************
*
*       _JLINKMEM_GetNextChar()
*/
static OS_INT _JLINKMEM_GetNextChar(void) {
  return OS_COM_GetNextChar();
}

/*********************************************************************
*
*       OS_COM_Init()
*       Initialize memory access for OSView
*/
void OS_COM_Init(void) {
  JLINKMEM_SetpfOnRx(_JLINKMEM_OnRx);
  JLINKMEM_SetpfOnTx(_JLINKMEM_OnTx);
  JLINKMEM_SetpfGetNextChar(_JLINKMEM_GetNextChar);
}

/*********************************************************************
*
*       OS_COM_Send1()
*       Send 1 character via memory
*/
void OS_COM_Send1(OS_U8 c) {
  JLINKMEM_SendChar(c);
}

#else // No UART communication for OS enabled, define dummy functions
/*********************************************************************
*
*       Communication for embOSView not selected
*
**********************************************************************
*/
void OS_COM_Init(void) {}
void OS_COM_Send1(OS_U8 c) {
  OS_USEPARA(c);
  OS_COM_ClearTxActive();  /* Free OS transmitter buffer */
}
#endif /*  OS_UART_USED  */

/****** Final check of configuration ********************************/

#ifndef OS_UART_USED
  #error "OS_UART_USED has to be defined"
#endif

/*********************************************************************
*
*       OS interrupt handler and ISR specific functions
*
**********************************************************************
*/

#if OS_USE_VARINTTABLE
  //
  // The interrupt vector table may be located anywhere in RAM
  //
  #ifdef __ICCARM__  // IAR
    #pragma data_alignment=512
    __no_init void (*g_pfnRAMVectors[NUM_INTERRUPTS]) (void);
  #endif  // __ICCARM__

  #ifdef __CC_ARM    // KEIL
    __attribute__ (zero_init, aligned(512)) void (*g_pfnRAMVectors[NUM_INTERRUPTS])(void);
  #endif

  #ifdef __GNUC__    // GCC
    void (*g_pfnRAMVectors[NUM_INTERRUPTS]) (void) __attribute__ ((aligned (512)));
  #endif
#endif

/*********************************************************************
*
*       OS_ARM_InstallISRHandler
*/
OS_ISR_HANDLER* OS_ARM_InstallISRHandler (int ISRIndex, OS_ISR_HANDLER* pISRHandler) {
#if OS_USE_VARINTTABLE
  OS_ISR_HANDLER*  pOldHandler;
  OS_U32           ulIdx;
  OS_U32*          pVect;

  pOldHandler = NULL;
  //
  // Check whether the RAM vector table has been initialized.
  //
  if ((*(OS_U32*)NVIC_VTOREG_ADDR) != (unsigned long)g_pfnRAMVectors) {
    //
    // Copy the vector table from the beginning of FLASH to the RAM vector table.
    //
    pVect = (OS_U32*)(*(OS_U32*)NVIC_VTOREG_ADDR);
    for(ulIdx = 0; ulIdx < NUM_INTERRUPTS; ulIdx++) {
      g_pfnRAMVectors[ulIdx] = (void (*)(void))(pVect[ulIdx]);
    }
    //
    // Program NVIC to point at the RAM vector table.
    //
    *(OS_U32*)NVIC_VTOREG_ADDR = (OS_U32)g_pfnRAMVectors;
  }
  //
  // Save the interrupt handler.
  //
  pOldHandler = g_pfnRAMVectors[ISRIndex];
  g_pfnRAMVectors[ISRIndex] = pISRHandler;
  return (pOldHandler);
#else
  //
  // The function does nothing if vector table is constant
  //
  OS_USEPARA(ISRIndex);
  OS_USEPARA(pISRHandler);
  return (NULL);
#endif
}

/*********************************************************************
*
*       OS_ARM_EnableISR
*/
void OS_ARM_EnableISR(int ISRIndex) {
  OS_DI();
  if (ISRIndex < NUM_INTERRUPTS) {
    if (ISRIndex >= 16) {
      //
      // Enable standard "external" interrupts, starting at index 16
      //
      ISRIndex -= 16;
      *(((OS_U32*) NVIC_ENABLE_ADDR) + (ISRIndex >> 5)) = (1uL << (ISRIndex & 0x1F));
    } else if (ISRIndex == ISR_ID_MPU) {
      //
      // Enable the MemManage interrupt.
      //
      SYSHND_CTRL |= NVIC_SYS_HND_CTRL_MEM;
    } else if (ISRIndex == ISR_ID_BUS) {
      //
      // Enable the bus fault interrupt.
      //
      SYSHND_CTRL |= NVIC_SYS_HND_CTRL_BUS;
    } else if (ISRIndex == ISR_ID_USAGE) {
      //
      // Enable the usage fault interrupt.
      //
      SYSHND_CTRL |= NVIC_SYS_HND_CTRL_USAGE;
    } else if (ISRIndex == ISR_ID_SYSTICK) {
      //
      // Enable the System Tick interrupt.
      //
      SYS_TICK_CONTROL |= (1uL << SYS_TICK_INT_ENABLE_BIT);
    }
  }
  OS_RestoreI();
}

/*********************************************************************
*
*       OS_ARM_DisableISR
*/
void OS_ARM_DisableISR(int ISRIndex) {
  OS_DI();
  if (ISRIndex < NUM_INTERRUPTS) {
    if (ISRIndex >= 16) {
      //
      // Disable standard "external" interrupts
      //
      ISRIndex -= 16;
      *(((OS_U32*) NVIC_DISABLE_ADDR) + (ISRIndex >> 5)) = (1uL << (ISRIndex & 0x1F));
    } else if (ISRIndex == ISR_ID_MPU) {
      //
      // Disable the MemManage interrupt.
      //
      SYSHND_CTRL &= ~NVIC_SYS_HND_CTRL_MEM;
    } else if (ISRIndex == ISR_ID_BUS) {
      //
      // Disable the bus fault interrupt.
      //
      SYSHND_CTRL &= ~NVIC_SYS_HND_CTRL_BUS;
    } else if (ISRIndex == ISR_ID_USAGE) {
      //
      // Disable the usage fault interrupt.
      //
      SYSHND_CTRL &= ~NVIC_SYS_HND_CTRL_USAGE;
    } else if (ISRIndex == ISR_ID_SYSTICK) {
      //
      // Enable the System Tick interrupt.
      //
      SYS_TICK_CONTROL &= ~(1uL << SYS_TICK_INT_ENABLE_BIT);
    }
  }
  OS_RestoreI();
}

/*********************************************************************
*
*       OS_ARM_ISRSetPrio
*
*   Notes:
*     (1) Some priorities of system handler are reserved
*         0..3 : Priority can not be set
*         7..10: Reserved
*         13   : Reserved
*     (2) System handler use different control register. This affects
*         ISRIndex 0..15
*/
int OS_ARM_ISRSetPrio(int ISRIndex, int Prio) {
  OS_U8* pPrio;
  int    OldPrio;

  OldPrio = 0;
  if (ISRIndex < NUM_INTERRUPTS) {
    if (ISRIndex >= 16) {
      //
      // Handle standard "external" interrupts
      //
      ISRIndex -= 16;                   // Adjust index
      OS_DI();
      pPrio = (OS_U8*)(NVIC_PRIOBASE_ADDR + ISRIndex);
      OldPrio = *pPrio;
      *pPrio = Prio;
      OS_RestoreI();
    } else {
      //
      // Handle System Interrupt controller
      //
      if ((ISRIndex < 4) | ((ISRIndex >= 7) && (ISRIndex <= 10)) | (ISRIndex == 13)) {
        //
        // Reserved ISR channel, do nothing
        //
      } else {
        //
        // Set priority in system interrupt priority control register
        //
        OS_DI();
        pPrio = (OS_U8*)(SYSPRI1_ADDR);
        ISRIndex -= 4;                  // Adjust Index
        OldPrio = pPrio[ISRIndex];
        pPrio[ISRIndex] = Prio;
        OS_RestoreI();
      }
    }
  }
  return OldPrio;
}

/*****  End Of File  ************************************************/
