#include "stm32f10x.h"
#include "one_wire.h"
#include "18B20.h"

float t_18b20;

void convertDs18b20( void ) { 
   resetOnewire(); 
   wOnewire(0xcc); 
   wOnewire(0x44); 
}

u8* readID( void ) { 
   u8 ID[8], i, *p;
   resetOnewire(); 
   wOnewire(0x33);
   for ( i = 0; i < 8; i++ ) {
      ID[i] = rOnewire();
   }
   p=ID;
   return p;
}

u16 readTemp( void ) { 
	u8 temp1, temp2;
	convertDs18b20();
	resetOnewire(); 
	wOnewire(0xcc); 
	wOnewire(0xbe); 	
	temp1 = rOnewire(); 
	temp2 = rOnewire(); 
#if 0   
	temp2 = temp2<<4;
	temp1 = temp1>>4;
	temp2 |= temp1;
	return ( temp2 & 0x7F );
#else
	return ( ( (u16)temp2 << 8 ) | temp1 );   
#endif   
}
