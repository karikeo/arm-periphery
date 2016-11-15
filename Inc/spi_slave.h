#include "RTOS.h"
#include "BSP.h"

#define SWAP16(x) ( (x >> 8) | (x << 8) )

#define SPI_TASK_EVT_RX       (1<<1)
#define SPI_TASK_EVT          (1<<2)
#define SPI_TASK_EVT_RX_SYNC  (1<<3)
#define SPI_TASK_EVT_100MS    (1<<4)

void spi_slave_init(void);
void Task_spi(void);
__irq void SPI2_IRQHandler(void);
__irq void DMA1_Channel4_IRQHandler(void);
__irq void DMA1_Channel5_IRQHandler(void);
//__irq void EXTI15_10_IRQHandler(void);
//__irq void HardFault_Handler(void);

#ifdef __ICCARM__  // IAR
   #define disable_interrupt() __disable_interrupt()
   #define enable_interrupt() __enable_interrupt()
#elif __CC_ARM    // KEIL
   #define disable_interrupt() __disable_irq()
   #define enable_interrupt() __enable_irq()  
#elif __GNUC__    // GCC
  #warning watch me
#endif




void timer100_cb( void );
void spi_ok_timer_cb( void );
void spi_sync_timer_cb( void );
void configure_spi(void);
void hookFunc( void );



