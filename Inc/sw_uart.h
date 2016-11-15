#define TX_H GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define TX_L GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define RX GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

void sw_uart_init(void);
void swuart_isr_tx(void);
void swuart_isr_rx(void);
__irq void TIM4_IRQHandler(void);
