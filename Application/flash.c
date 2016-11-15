#include "stm32f10x.h"
#include "RTOS.h"
#include "flash.h"
#include "setjmp.h"

/* Flash Control Register bits */
#define CR_PG_Set                ((u32)0x00000001)
#define CR_PG_Reset              ((u32)0x00001FFE) 

#define CR_PER_Set               ((u32)0x00000002)
#define CR_PER_Reset             ((u32)0x00001FFD)

#define CR_MER_Set               ((u32)0x00000004)
#define CR_MER_Reset             ((u32)0x00001FFB)

#define CR_OPTPG_Set             ((u32)0x00000010)
#define CR_OPTPG_Reset           ((u32)0x00001FEF)

#define CR_OPTER_Set             ((u32)0x00000020)
#define CR_OPTER_Reset           ((u32)0x00001FDF)

#define CR_STRT_Set              ((u32)0x00000040)
							 
#define CR_LOCK_Set              ((u32)0x00000080)


#define FLASH_KEY1               ((u32)0x45670123)
#define FLASH_KEY2               ((u32)0xCDEF89AB)

/* Delay definition */   
#define EraseTimeout             ((u32)0x00000FFF)
#define ProgramTimeout           ((u32)0x0000000F)

extern void __iar_program_start(void);

volatile u32 ep;//__iar_program_start;

#if 0
#pragma location = ".bootloader"
void flash_unlock( ) {
   if ( ( flash->CR & CR_LOCK_Set ) == CR_LOCK_Set ) {
      flash->KEYR = FLASH_KEY1;
      flash->KEYR = FLASH_KEY2;     
   }  
}
#endif

#ifdef __ICCARM__  // IAR
   #pragma location = ".bootloader"
#endif
#ifdef __CC_ARM    // KEIL
   #pragma arm section code = ".bootloader"
#endif
#ifdef __GNUC__    // GCC
  #warning no segment specified
#endif

void fwupdate( u8* pData, u16 size, FLASH_TypeDef* flash ) {

   //static FLASH_Status st;
   static u32 i = 0;      
   
//   if ( !pData || size == 0 )
//     return;
     
   if ( ( flash->CR & CR_LOCK_Set ) == CR_LOCK_Set ) {
      flash->KEYR = FLASH_KEY1;
      flash->KEYR = FLASH_KEY2;     
   }
   
#if 0   
   volatile u32* volatile p = (u32*) 0x08000004;
   ep = *p;
#endif     
   
   //st = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
   for ( i = 0; i < 0xFFFF; i++ ) {
      if ( ( flash->SR & FLASH_FLAG_BSY ) != FLASH_FLAG_BSY ) {
         break;
      }
   }

#if 0  
   if ( st == FLASH_COMPLETE )  { 
    /* if the previous operation is completed, proceed to erase the page */
      flash->CR |= CR_PER_Set;
      flash->AR = (u32)pData; 
      flash->CR |= CR_STRT_Set;
    
      /* Wait for last operation to be completed */      
      for ( i = 0; i < 0xFFFF; i++ ) {
         if ( ( flash->SR & FLASH_FLAG_BSY ) != FLASH_FLAG_BSY ) {
            break;
         }
      }

      if ( st != FLASH_BUSY ) {
         /* if the erase operation is completed, disable the PER Bit */
         flash->CR &= CR_PER_Reset;
      }
   }   
   
#endif
   
   flash->CR |= CR_LOCK_Set;   

#if 0   
   NVIC_GenerateSystemReset();
#endif
   
#if 0      
   __asm("    mov     r0, sp\n"
         "    bl      ep\n"
         "    bx      lr");   
#endif   
      
}
