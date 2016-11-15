#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "lcd_2x16.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

OS_RSEMA SemaLCD;

extern GPIO_InitTypeDef GPIO_InitStructure;
static vu32 TimingDelay;


//unsigned char lcd_buf[ 32 ];

/****************************/
/* Strobe 4-Bit Data to LCD */
/****************************/
void lcd_out_data4(unsigned char val)
{
  if((val&0x01)==0x01)	// Bit[0]
  {
    LCD_D4_HI();
  }
  else
  {
    LCD_D4_LO();
  }

  if((val&0x02)==0x02)  // Bit[1]
  {
    LCD_D5_HI();
  }
  else
  {
    LCD_D5_LO();
  }

  if((val&0x04)==0x04)  // Bit[2]
  {
    LCD_D6_HI();
  }
  else
  {
    LCD_D6_LO();
  }

  if((val&0x08)==0x08)  // Bit[3]
  {
    LCD_D7_HI();
  }
  else
  {
    LCD_D7_LO();
  }

}

/****************************/
/* Write Data 1 Byte to LCD */
/****************************/
void lcd_write_byte(unsigned char val)
{
  lcd_out_data4((val>>4)&0x0F);							// Strobe 4-Bit High-Nibble to LCD
  enable_lcd();											// Enable Pulse

  lcd_out_data4(val&0x0F);				  				// Strobe 4-Bit Low-Nibble to LCD
  enable_lcd();											// Enable Pulse

  while(busy_lcd());      								// Wait LCD Execute Complete
}

/****************************/
/* Write Instruction to LCD */
/****************************/
void lcd_write_control(unsigned char val)
{
  LCD_RS_LO();											// RS = 0 = Instruction Select
  lcd_write_byte(val);									// Strobe Command Byte
}

/****************************/
/* Write Data(ASCII) to LCD */
/****************************/
void lcd_write_ascii(unsigned char c)
{
  LCD_RS_HI();											// RS = 1 = Data Select
  lcd_write_byte(c);		   							// Strobe 1 Byte to LCD
}

/*******************************/
/* Initial 4-Bit LCD Interface */
/*******************************/
void lcd_init(void)
{
  /* Configure IO connected to LCD16X2 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_RW, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LCD_EN_PIN;
  GPIO_Init((GPIO_TypeDef*)GPIOC_BASE, &GPIO_InitStructure);

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_RW, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LCD_RW_PIN;
  GPIO_Init((GPIO_TypeDef*)GPIOC_BASE, &GPIO_InitStructure);

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_RS, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
  GPIO_Init((GPIO_TypeDef*)GPIOC_BASE, &GPIO_InitStructure);

//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DATA, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = LCD_D4_PIN |
                                LCD_D5_PIN |
				LCD_D6_PIN |
                                LCD_D7_PIN;
  GPIO_Init((GPIO_TypeDef*)GPIOC_BASE, &GPIO_InitStructure);

  LCD_D4_HI();
  LCD_D5_HI();
  LCD_D6_LO();
  LCD_D7_LO();
  DelayuS(15000);										// Power-On Delay (15 mS)

  LCD_D4_HI();
  LCD_D5_HI();
  LCD_D6_LO();
  LCD_D7_LO();
  enable_lcd();											// Enable Pulse
  DelayuS(4100);										// Delay 4.1mS

  LCD_D4_HI();
  LCD_D5_HI();
  LCD_D6_LO();
  LCD_D7_LO();
  enable_lcd();											// Enable Pulse
  DelayuS(100);											// delay 100uS

  LCD_D4_HI();
  LCD_D5_HI();
  LCD_D6_LO();
  LCD_D7_LO();
  enable_lcd();											// Enable Pulse
  //while(busy_lcd());      								// Wait LCD Execute Complete
   DelayuS(1000);

  LCD_D4_LO();
  LCD_D5_HI();
  LCD_D6_LO();
  LCD_D7_LO();
  enable_lcd();											// Enable Pulse
  //while(busy_lcd());      								// Wait LCD Execute Complete
   DelayuS(1000);

  lcd_write_control(0x28);  							// Function Set (DL=0 4-Bit,N=1 2 Line,F=0 5X7)
  lcd_write_control(0x0C);  							// Display on/off Control (Entry Display,Cursor off,Cursor not Blink)
  lcd_write_control(0x06);  							// Entry Mode Set (I/D=1 Increment,S=0 Cursor Shift)
  lcd_write_control(0x01);  							// Clear Display  (Clear Display,Set DD RAM Address=0)
  //DelaymS(15);  										// Wait Command Ready
  //OS_Delay(15);
  DelayuS(15000);
}

/***************************/
/* Set LCD Position Cursor */
/***************************/
void goto_cursor(unsigned char i)
{
  i |= 0x80;											// Set DD-RAM Address Command
#if defined (BOOTLOADER)
  lcd_write_control(i);
#else
  OS_Use( &SemaLCD );
  lcd_write_control(i);
  OS_Unuse( &SemaLCD );
#endif
}

void lcd_clear() {
#if defined (BOOTLOADER)
   lcd_write_control(0x01);	// Clear Display
#else
   OS_Use( &SemaLCD );
   lcd_write_control(0x01);	// Clear Display
   OS_Unuse( &SemaLCD );
#endif
}

/************************************/
/* Print Display Data(ASCII) to LCD */
/************************************/
void lcd_print( char* str )
{
  int i;
#if !defined (BOOTLOADER)
  OS_Use( &SemaLCD );
#endif
  for (i=0;i<16 && str[i]!=0;i++)  						// 16 Character Print
  {
    lcd_write_ascii(str[i]);							// Print Byte to LCD
  }
   goto_cursor( 0x00 );
#if !defined (BOOTLOADER)
   OS_Unuse( &SemaLCD );
#endif
}

/******************/
/* Wait LCD Ready */
/******************/

char busy_lcd(void)
{
#if !defined (BOOTLOADER)
   if ( OS_IsRunning() ) {
      OS_Delay (1);
   } else {
      DelayuS(100);
   }
   IWDG_ReloadCounter();
#else
   DelayuS( 1000 );
#endif
   return 0;											// LCD Busy Status

#if 0
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// Config D7 = Read
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = LCD_D7_PIN;
  GPIO_Init((GPIO_TypeDef*)LCD_DATA_PORT, &GPIO_InitStructure);

  LCD_RS_LO();		 									// Instruction Select
  LCD_RW_HI(); 											// Read Direction
  LCD_EN_HI();											// Start Read Busy

  DelayuS(100);
  //OS_Delay (1);
  // Delay Before Read
  if (GPIO_ReadInputDataBit((GPIO_TypeDef*)LCD_DATA_PORT, LCD_D7_PIN) == Bit_SET)
  {
    LCD_EN_LO();  										// Disable Read
  	LCD_RW_LO();										// Default = Write Direction

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  	// Config D7 = Write
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = LCD_D7_PIN;
    GPIO_Init((GPIO_TypeDef*)LCD_DATA_PORT, &GPIO_InitStructure);
    return 1;											// LCD Busy Status
  }
  else
  {
    LCD_EN_LO();  										// Disable Read
  	LCD_RW_LO();										// Default = Write Direction

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// Config D7 = Write
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = LCD_D7_PIN;
    GPIO_Init((GPIO_TypeDef*)LCD_DATA_PORT, &GPIO_InitStructure);
    return 0;											// LCD Ready Status
  }
#endif  
}


/***********************/
/* Enable Pulse to LCD */
/***********************/
void enable_lcd(void)	 								// Enable Pulse
{
  LCD_EN_HI();  										// Enable ON
  DelayuS(50);
  //OS_Delay(1);
  LCD_EN_LO();  										// Enable OFF
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void DelayuS(vu32 nCount)
{
  while (nCount--);
}

#if 0
/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nTime: specifies the delay time length, in milliseconds.
* Output         : None
* Return         : None
*******************************************************************************/
void DelaymS(u32 nTime)
{
  /* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  TimingDelay = nTime;

  while(TimingDelay != 0);

  /* Disable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);
  /* Clear SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}
#endif

#if 1
/*******************************************************************************
* Function Name  : TimingDelay_Decrement
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

#if  0
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
#endif
