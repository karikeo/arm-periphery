  // Define LCD-16x2 PinIO Interface Mask Bit
  #define  LCD_EN_PIN		           GPIO_Pin_10			// EN = PC[10]
  #define  LCD_EN_PORT		           GPIOC_BASE
  #define  RCC_APB2Periph_GPIO_EN      RCC_APB2Periph_GPIOC

  #define  LCD_RW_PIN		           GPIO_Pin_11			// RW = PC[11]
  #define  LCD_RW_PORT	               GPIOC_BASE
  #define  RCC_APB2Periph_GPIO_RW      RCC_APB2Periph_GPIOC

  #define  LCD_RS_PIN 		       	   GPIO_Pin_12			// RS= PC[12]
  #define  LCD_RS_PORT		           GPIOC_BASE
  #define  RCC_APB2Periph_GPIO_RS      RCC_APB2Periph_GPIOC

  #define  LCD_D4_PIN 		       	   GPIO_Pin_3			// D4 = PC[3]
  #define  LCD_D5_PIN 		       	   GPIO_Pin_2			// D5 = PC[2]
  #define  LCD_D6_PIN 		       	   GPIO_Pin_1			// D6 = PC[1]
  #define  LCD_D7_PIN 		       	   GPIO_Pin_0			// D7 = PC[0]
  #define  LCD_DATA_PORT	           GPIOC_BASE
  #define  RCC_APB2Periph_GPIO_DATA    RCC_APB2Periph_GPIOC

  #define  LCD_EN_HI()    	           GPIO_WriteBit((GPIO_TypeDef *)LCD_EN_PORT,LCD_EN_PIN,Bit_SET)
  #define  LCD_EN_LO()		           GPIO_WriteBit((GPIO_TypeDef *)LCD_EN_PORT,LCD_EN_PIN,Bit_RESET)

  #define  LCD_RW_HI()    	           GPIO_WriteBit((GPIO_TypeDef *)LCD_RW_PORT,LCD_RW_PIN,Bit_SET)
  #define  LCD_RW_LO() 		           GPIO_WriteBit((GPIO_TypeDef *)LCD_RW_PORT,LCD_RW_PIN,Bit_RESET)

  #define  LCD_RS_HI() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_RS_PORT,LCD_RS_PIN,Bit_SET)
  #define  LCD_RS_LO() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_RS_PORT,LCD_RS_PIN,Bit_RESET)

  #define  LCD_D4_HI() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D4_PIN,Bit_SET)
  #define  LCD_D4_LO() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D4_PIN,Bit_RESET)

  #define  LCD_D5_HI() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D5_PIN,Bit_SET)
  #define  LCD_D5_LO() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D5_PIN,Bit_RESET)

  #define  LCD_D6_HI() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D6_PIN,Bit_SET)
  #define  LCD_D6_LO() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D6_PIN,Bit_RESET)

  #define  LCD_D7_HI() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D7_PIN,Bit_SET)
  #define  LCD_D7_LO() 	               GPIO_WriteBit((GPIO_TypeDef *)LCD_DATA_PORT,LCD_D7_PIN,Bit_RESET)

  //#define  lcd_clear()                 lcd_write_control(0x01)	// Clear Display
  #define  lcd_cursor_home()           lcd_write_control(0x02)	// Set Cursor = 0
  #define  lcd_display_on()            lcd_write_control(0x0E)	// LCD Display Enable
  #define  lcd_display_off()           lcd_write_control(0x08)	// LCD Display Disable
  #define  lcd_cursor_blink()          lcd_write_control(0x0F)	// Set Cursor = Blink
  #define  lcd_cursor_on()             lcd_write_control(0x0E)	// Enable LCD Cursor
  #define  lcd_cursor_off()            lcd_write_control(0x0C)	// Disable LCD Cursor
  #define  lcd_cursor_left()           lcd_write_control(0x10)	// Shift Left Cursor
  #define  lcd_cursor_right()          lcd_write_control(0x14)	// Shift Right Cursor
  #define  lcd_display_sleft()         lcd_write_control(0x18)	// Shift Left Display
  #define  lcd_display_sright()        lcd_write_control(0x1C)	// Shift Right Display


void lcd_init(void);					// Initial LCD
void lcd_out_data4(unsigned char);		// Strobe 4-Bit Data to LCD
void lcd_write_byte(unsigned char);		// Write 1 Byte Data to LCD
void lcd_write_control(unsigned char); 	// Write Instruction
void lcd_write_ascii(unsigned char); 	// Write LCD Display(ASCII)
void goto_cursor(unsigned char);		// Set Position Cursor LCD
void lcd_print( char*);			// Print Display to LCD
char busy_lcd(void);					// Read Busy LCD Status
void enable_lcd(void);	 				// Enable Pulse
void DelayuS(vu32 nCount);	 			// 1uS Delay
//void DelaymS(vu32 nTime);				// 1mS Delay
void TimingDelay_Decrement(void);

void lcd_clear(void);
//extern unsigned char lcd_buf[ 32 ];
