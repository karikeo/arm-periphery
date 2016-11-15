#ifndef _SWUART_
#define _SWUART_

#include "RTOS.h"
#include "BSP.h"

#define TX_H GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define TX_L GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define RX GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

typedef struct 
{
  struct 
  {
    u8 size;
    u8 pos;
    u8 buf[32];
  } tx;
  struct     
  {
    u8 size;    
    u8 buf[32];
    u8 mes;
    void (*handler) (void);
  } rx;
} t_sw_uart;

void sw_uart_init(void);
void swuart_isr_tx(void);
void swuart_isr_rx(void);
__irq void TIM4_IRQHandler(void);
void sw_uart_tx(u8* p, int size);
void sw_uart_reset_buffers(void);

#endif
