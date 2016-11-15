#ifndef _18B20_H
#define _18B20_H

//#include "one_wire.h"

void convertDs18b20(void);
u8* readID(void);
u16 readTemp(void);

#endif /*_18B20_H*/

