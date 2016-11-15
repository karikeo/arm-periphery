#include "stm32f10x.h"
#include "stm32f10x_dbgmcu.h"
#include "RTOS.h"
#include "BSP.h"
//#include "stdio.h"
//#include "log.h"
//#include "vending/pt-vending.h"
//#include "vending/dex.h"
//#include "vending/mdb.h"
#include "utils.h"
#include "sw_uart.h"

extern GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
TIM_ICInitTypeDef TIM_ICInitStruct;
TIM_OCInitTypeDef TIM_OCInitStruct;

t_sw_uart sw_uart;

void sw_uart_init()
{
  //sw_uart.tx.buf[0] = 0xFF;
  //sw_uart.tx.size = 1;
  sw_uart.tx.pos = 0;   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  TX_H;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitStruct.TIM_Period = (1000000/9523)-1;
  //TIM_TimeBaseInitStruct.TIM_Period = (1000000/9600)-1;  
  TIM_TimeBaseInitStruct.TIM_Prescaler = 36-1;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

  TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStruct.TIM_ICFilter = 0;
  TIM_ICInit(TIM4, &TIM_ICInitStruct);

  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE );
  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE );   
  TIM_Cmd(TIM4, ENABLE);
  OS_ARM_EnableISR( TIM4_IRQn + 16 );
}


void sw_uart_tx(u8* p, int size)
{
  sw_uart.tx.pos = 0;
  memcpy( sw_uart.tx.buf, p, size );
  sw_uart.tx.size = size;  
}

u8 pin8 = 0;

void swuart_isr_tx()
{
#if 0
  if ( pin8 )
  {
    TX_H;
  }
  else
  {
    TX_L;
  }
  pin8 = !pin8;
#else  
  
  static u8 st = 0;
  static u8 b = 0;
  static u8 bit_no = 0;  
  switch( st )
  {
    case 0:
      if ( sw_uart.tx.pos < sw_uart.tx.size )
      {
        b = sw_uart.tx.buf[ sw_uart.tx.pos ];
        bit_no = 0;
        TX_L;
        st++;
        return;
      }
      else if ( sw_uart.tx.pos > 0 )
      {
        sw_uart.tx.pos = 0;
        sw_uart.tx.size = 0;
      }
      break;
     
    case 2:
    case 4:
    case 6:      
    case 8:
    case 10:
    case 12:
    case 14:      
    case 16:      
      ( b & ( 0x01 << bit_no++ ) ) ? TX_H : TX_L;
      break;
      
    case 18:
      TX_H;
      break;
      
    case 20:
      sw_uart.tx.pos++;
      //sw_uart.tx.buf[0]++;
      break;      
  }
      
  if ( st >= 20 )  
  {
    st = 0;
  }
  else if ( st != 0 )
  {
    st++;
  }
#endif  
}

void swuart_isr_rx(void)
{
  u8 bit = RX;
  u8 bit1 = RX;
  u8 bit2 = RX;
  u8 done = 0;
  u8 framing_err = 0;  
  static u8 st = 0;
  static u8 b = 0;
  static u8 bit_no = 0;  
  
  if ( bit != bit1 || bit != bit2 )
  {
    b = 0;
  }
  
  switch( st )
  {
    case 0:
      TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_Timing;
      TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Disable;
      TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
      TIM_OCInitStruct.TIM_Pulse = TIM4->CCR4;
      TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
      TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
      TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
      TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
      TIM_OC4Init(TIM4, &TIM_OCInitStruct);
      st++;
      return;
      
    case 1:
      if ( !bit )
      {//start bit
        b = 0;
        bit_no = 0;        
      }
      else
      {//framimg error, no start bit
        framing_err = 1;
      }
      break;
  
    case 3 + 0:
    case 3 + 2:
    case 3 + 4:
    case 3 + 6:
    case 3 + 8:
    case 3 + 10:
    case 3 + 12:
    case 3 + 14:      
      if ( bit )
      {
        b |= ( 1 << bit_no );
      }
      bit_no++;
      break;
      
    case 3 + 16:
      if ( bit )
      {
        if ( sw_uart.rx.size < sizeof(sw_uart.rx.buf)-1)
        {
          sw_uart.rx.buf[ sw_uart.rx.size++ ] = b;       
          if ( b == sw_uart.rx.mes && sw_uart.rx.handler )
          {
            sw_uart.rx.handler();
          }
        }
        done = 1;
      }
      else
      {
        framing_err = 1;
      }
      break;
            
  }

  if ( done || framing_err )
  {
      st = 0;
      TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
      TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Falling;
      TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
      TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
      TIM_ICInitStruct.TIM_ICFilter = 0;
      TIM_ICInit(TIM4, &TIM_ICInitStruct);                  
  }
    
  if ( st > 0 && st < 100 )
  {
    st++;
  }
}

__irq void TIM4_IRQHandler(void) 
{
  //OS_EnterNestableInterrupt();
  if ( TIM_GetITStatus(TIM4, TIM_IT_Update ) )
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);    
    swuart_isr_tx();      
  }
  if ( TIM_GetITStatus(TIM4, TIM_IT_CC4 ) )
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4 );    
    swuart_isr_rx();      
  }              
  //OS_LeaveNestableInterrupt();
}

void sw_uart_reset_buffers()
{
  sw_uart.rx.size = 0;
  memset( sw_uart.rx.buf, 0, sizeof(sw_uart.rx.buf));
  sw_uart.tx.size = 0;
  memset( sw_uart.tx.buf, 0, sizeof(sw_uart.tx.buf));  
}
