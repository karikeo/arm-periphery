/*********************************************************************
--------- END-OF-HEADER --------------------------------------------*/
#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "log.h"
#include "spi_slave.h"
#include "flash.h"
#include "vending/pt-vending.h"
#include "18B20.h"

/* Task stacks */
OS_STACKPTR int Stack1[256], Stack2[256], Stack3[256]/*, Stack4[256]*/;
/* Task-control-blocks */
OS_TASK TCB1, TCB2, TCB3, TCB4;

static void Task4(void) {
   extern float t_18b20;
   while (1) {
      float f;
      u16 u;
      OS_Delay (1000);
      u = readTemp();
      f = (float)u;
      f /= 16.0;
      //log( "t: %.1fC\r\n", f );
      t_18b20 = f;
   }
}

/*********************************************************************
*
*       main
*
*********************************************************************/

int main(void)
{
	OS_IncDI();                      /* Initially disable interrupts  */
	OS_InitKern();                   /* Initialize OS                 */
	OS_InitHW();                     /* Initialize Hardware for OS    */
	BSP_Init();                      /* Initialize LED ports          */
	OS_CREATETASK(&TCB1, "Task SPI", Task_spi,  50, Stack1);
	OS_CREATETASK(&TCB2, "Task Vending", Task_vending,  51, Stack2);
	OS_CREATETASK(&TCB3, "USART3", Task_USART3,  52, Stack3);
	//OS_CREATETASK(&TCB4, "Task4", Task4,  53, Stack4);
	OS_Start();                      /* Start multitasking            */
	return 0;
}

